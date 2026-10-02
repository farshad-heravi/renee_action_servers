#!/usr/bin/env python3
"""Mock camera service implementing docs/camera_link_protocol.md (version 1).

Stands in for the Jetson ZED service so the PC-side capture_camera_frames action
server can be tested without hardware. Stdlib only, Python 3.6 compatible.

Fault injection (--fault):
  none       well-behaved service
  stale      frame 0 of every capture has a capture time before the request arrived
  latency    every request is answered --latency-ms late (inflates the measured RTT)
  drop       frame index --drop-index is never sent (gap in the indices)
  clock_bad  pong reports clock_ok=false
--clock-skew-s adds a constant offset to this service's clock.
"""

import argparse
import json
import select
import socket
import struct
import sys
import threading
import time
from array import array

PROTOCOL_VERSION = 1


def now_ns(skew_ns):
    return int(time.time() * 1e9) + skew_ns


def recv_exact(sock, count):
    chunks = []
    while count > 0:
        chunk = sock.recv(count)
        if not chunk:
            raise ConnectionError('peer closed the connection')
        chunks.append(chunk)
        count -= len(chunk)
    return b''.join(chunks)


def read_message(sock):
    (header_len,) = struct.unpack('>I', recv_exact(sock, 4))
    header = json.loads(recv_exact(sock, header_len).decode('utf-8'))
    blobs = [recv_exact(sock, size) for size in header.get('blob_sizes', [])]
    return header, blobs


def send_message(sock, header, blobs=()):
    header = dict(header)
    header['version'] = PROTOCOL_VERSION
    header['blob_sizes'] = [len(blob) for blob in blobs]
    payload = json.dumps(header).encode('utf-8')
    sock.sendall(struct.pack('>I', len(payload)) + payload + b''.join(blobs))


class FrameSource(object):
    """Synthetic gradient RGB (moves with the frame index) and a static depth ramp."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self._x_ramp = [(x * 255) // width for x in range(width)]
        depth = array('H')
        for y in range(height):
            for x in range(width):
                if ((x // 40) + (y // 40)) % 7 == 0:
                    depth.append(0)  # invalid pixels
                else:
                    depth.append(600 + (x * 3000) // width + (y * 500) // height)
        if sys.byteorder == 'big':
            depth.byteswap()
        self._depth_compressed = zlib_compress(depth.tobytes())
        fx = 0.8 * width
        self.camera_info = {
            'width': width, 'height': height, 'distortion_model': 'plumb_bob',
            'D': [0.0, 0.0, 0.0, 0.0, 0.0],
            'K': [fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0],
            'R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'P': [fx, 0.0, width / 2.0, 0.0, 0.0, fx, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        }

    def rgb_compressed(self, index):
        shift = (index * 7) & 255
        blue = bytes(((value + shift) & 255) for value in self._x_ramp)
        red = bytes([(shift * 3) & 255]) * self.width
        raw = bytearray()
        for y in range(self.height):
            row = bytearray(3 * self.width)
            row[0::3] = blue
            row[1::3] = bytes([(y * 255) // self.height]) * self.width
            row[2::3] = red
            raw += row
        return zlib_compress(bytes(raw))

    def depth_compressed(self):
        return self._depth_compressed


def zlib_compress(buf):
    import zlib
    return zlib.compress(buf, 1)


class Service(object):
    def __init__(self, args):
        self.args = args
        self.skew_ns = int(args.clock_skew_s * 1e9)
        self.source = FrameSource(args.width, args.height)

    def now(self):
        return now_ns(self.skew_ns)

    def pong(self, header):
        bad_clock = self.args.fault == 'clock_bad'
        return {
            'type': 'pong', 't0': header.get('t0'), 't1': header['_t1'], 't2': self.now(),
            'clock_ok': not bad_clock,
            'chrony_offset_ms': 250.0 if bad_clock else 0.3,
        }

    def serve(self, conn):
        try:
            while True:
                header, _ = read_message(conn)
                if self.args.fault == 'latency':
                    time.sleep(self.args.latency_ms / 1000.0)
                header['_t1'] = self.now()
                if header.get('version') != PROTOCOL_VERSION:
                    send_message(conn, {'type': 'error', 'message': 'unsupported protocol version'})
                    continue
                kind = header.get('type')
                if kind == 'ping':
                    send_message(conn, self.pong(header))
                elif kind == 'capture':
                    self.capture(conn, header)
                elif kind == 'cancel':
                    pass  # nothing running
                else:
                    send_message(conn, {'type': 'error', 'message': 'unknown type %r' % (kind,)})
        except (ConnectionError, OSError):
            pass
        finally:
            conn.close()

    def capture(self, conn, header):
        mode = header.get('mode')
        count = header.get('num_frames')
        if mode not in ('rgb', 'rgbd'):
            send_message(conn, {'type': 'error', 'message': 'mode must be rgb or rgbd'})
            return
        if not isinstance(count, int) or count < 1:
            send_message(conn, {'type': 'error', 'message': 'num_frames must be >= 1'})
            return

        period = 1.0 / self.args.fps
        next_time = time.time() + period  # first frame is a fresh grab, one period away
        sent = 0
        cancelled = False
        for index in range(count):
            while not cancelled:
                remaining = next_time - time.time()
                if remaining <= 0:
                    break
                readable, _, _ = select.select([conn], [], [], remaining)
                if readable:
                    other, _ = read_message(conn)
                    if other.get('type') == 'cancel':
                        cancelled = True
                    elif other.get('type') == 'ping':
                        other['_t1'] = self.now()
                        send_message(conn, self.pong(other))
            if cancelled:
                break
            next_time += period

            capture_ts = self.now()
            if self.args.fault == 'stale' and index == 0:
                capture_ts = header['_t1'] - 500000000
            if self.args.fault == 'drop' and index == self.args.drop_index:
                continue

            frame = {
                'type': 'frame', 'index': index, 'capture_ts_ns': capture_ts,
                'rgb': {'width': self.args.width, 'height': self.args.height,
                        'encoding': 'bgr8', 'codec': 'zlib'},
                'camera_info': self.source.camera_info,
            }
            blobs = [self.source.rgb_compressed(index)]
            if mode == 'rgbd':
                frame['depth'] = {'width': self.args.width, 'height': self.args.height,
                                  'encoding': '16UC1', 'codec': 'zlib'}
                blobs.append(self.source.depth_compressed())
            send_message(conn, frame, blobs)
            sent += 1

        done = {'type': 'done', 'n_frames': sent}
        if cancelled:
            done['cancelled'] = True
        send_message(conn, done)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7788)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--clock-skew-s', type=float, default=0.0)
    parser.add_argument('--fault', choices=['none', 'stale', 'latency', 'drop', 'clock_bad'],
                        default='none')
    parser.add_argument('--latency-ms', type=float, default=300.0)
    parser.add_argument('--drop-index', type=int, default=1)
    args = parser.parse_args()

    service = Service(args)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(4)
    print('mock camera service on %s:%d (%dx%d @ %.1f fps, fault=%s, skew=%.3f s)' % (
        args.host, args.port, args.width, args.height, args.fps, args.fault,
        args.clock_skew_s), flush=True)
    try:
        while True:
            conn, _ = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=service.serve, args=(conn,), daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == '__main__':
    main()
