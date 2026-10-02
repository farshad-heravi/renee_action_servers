#!/usr/bin/env python3
"""rclpy test client for the /capture_camera_frames action.

Single goal:
  test_capture_client.py --mode rgbd --num-frames 5 --topic-prefix /capture_camera_frames_action_server/

Scenario list from the verification plan (spawns scripts/mock_camera_service.py itself,
one instance per scenario, on --mock-port, which must equal the server's link_port):
  test_capture_client.py --scenarios              # all
  test_capture_client.py --scenarios rgb_n1 stale # a subset
  test_capture_client.py --scenarios --external-mock   # you run the mock; fault scenarios skipped

Uses small rclpy loops instead of `ros2 action` / `ros2 topic echo`, which hang here.
"""

import argparse
import math
import os
import socket
import subprocess
import sys
import time
from array import array

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from renee_action_servers.action import CaptureCameraFrames

DEFAULT_PREFIX = '/capture_camera_frames_action_server/'
MOCK_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mock_camera_service.py')
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def depth_stats(image):
    """Returns (valid_fraction, min, max) of the finite, non-zero depth values."""
    if not image.data:
        return None
    if image.encoding == '32FC1':
        values = array('f')
        values.frombytes(bytes(image.data))
        valid = [v for v in values if math.isfinite(v)]
    elif image.encoding == '16UC1':
        values = array('H')
        values.frombytes(bytes(image.data))
        valid = [v for v in values if v > 0]
    else:
        return None
    if not valid:
        return 0.0, None, None
    return len(valid) / len(values), min(valid), max(valid)


class CaptureClient(Node):
    def __init__(self, action_name, topic_prefix):
        super().__init__('capture_camera_frames_test_client')
        self.client = ActionClient(self, CaptureCameraFrames, action_name)
        self.topic_stamps = {'rgb': [], 'depth': [], 'info': []}
        self.phases = []
        if topic_prefix is not None:
            # Match the server's reliable publishers; a deep queue because rclpy handles
            # ~1 MB image messages slowly and a best-effort depth-5 queue drops frames.
            qos_profile_sensor_data = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(
                Image, topic_prefix + 'rgb/image_raw', self._on_topic('rgb'), qos_profile_sensor_data)
            self.create_subscription(
                Image, topic_prefix + 'depth/image_raw', self._on_topic('depth'),
                qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, topic_prefix + 'camera_info', self._on_topic('info'),
                qos_profile_sensor_data)

    def _on_topic(self, key):
        def callback(msg):
            self.topic_stamps[key].append(to_sec(msg.header.stamp))
        return callback

    def spin_until(self, predicate, timeout):
        deadline = time.time() + timeout
        while not predicate() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return predicate()

    def send_goal(self, mode, num_frames, timeout_sec, goal_wait=10.0):
        """Returns (goal_handle or None, info string)."""
        if not self.client.wait_for_server(timeout_sec=goal_wait):
            return None, 'action server not available'
        for stamps in self.topic_stamps.values():
            del stamps[:]
        del self.phases[:]
        goal = CaptureCameraFrames.Goal()
        goal.mode = mode
        goal.num_frames = num_frames
        goal.timeout_sec = float(timeout_sec)
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda fb: self.phases.append(fb.feedback.phase))
        if not self.spin_until(future.done, goal_wait):
            return None, 'timed out sending the goal'
        handle = future.result()
        if not handle.accepted:
            return None, 'goal rejected'
        return handle, 'accepted'

    def wait_result(self, handle, timeout, cancel_after=None):
        """Returns (status, result) or (None, None) on timeout."""
        result_future = handle.get_result_async()
        start = time.time()
        cancelled = False
        while not result_future.done() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cancel_after is not None and not cancelled and time.time() - start >= cancel_after:
                handle.cancel_goal_async()
                cancelled = True
        if not result_future.done():
            return None, None
        wrapped = result_future.result()
        return wrapped.status, wrapped.result

    def run_goal(self, mode, num_frames, timeout_sec, wait=60.0, cancel_after=None):
        t_sent = time.time()
        handle, info = self.send_goal(mode, num_frames, timeout_sec)
        if handle is None:
            return {'ok': False, 'info': info}
        status, result = self.wait_result(handle, wait, cancel_after)
        t_done = time.time()
        if status is None:
            return {'ok': False, 'info': 'timed out waiting for the result'}
        # Let late topic messages arrive before they are counted.
        self.spin_until(lambda: False, 0.3)
        return {'ok': True, 'status': status, 'result': result, 't_sent': t_sent,
                't_done': t_done, 'phases': list(self.phases),
                'topics': {k: list(v) for k, v in self.topic_stamps.items()}}


def summarize(run, num_frames):
    if not run['ok']:
        print('FAILED: %s' % run['info'])
        return
    res = run['result']
    print('status: %s' % STATUS_NAMES.get(run['status'], run['status']))
    print('success=%s captured=%d timestamps_valid=%s' % (
        res.success, res.captured_frames, res.timestamps_valid))
    print('message: %s' % res.message)
    print('clock_offset_ms=%.3f round_trip_ms=%.3f' % (res.clock_offset_ms, res.round_trip_ms))
    print('stamp=%.6f (goal sent %.6f, result %.6f)' % (
        to_sec(res.stamp), run['t_sent'], run['t_done']))
    for name, image in (('rgb', res.rgb), ('depth', res.depth)):
        if image.data:
            print('%s: %dx%d %s, %d bytes, stamp %.6f' % (
                name, image.width, image.height, image.encoding, len(image.data),
                to_sec(image.header.stamp)))
        else:
            print('%s: (empty)' % name)
    stats = depth_stats(res.depth)
    if stats:
        print('depth valid fraction %.3f, range %s .. %s' % stats)
    print('camera_info: %dx%d frame_id=%r fx=%.1f' % (
        res.camera_info.width, res.camera_info.height, res.camera_info.header.frame_id,
        res.camera_info.k[0]))
    stamps = [to_sec(s) for s in res.stamps]
    if stamps:
        deltas = ['%.0f' % ((b - a) * 1e3) for a, b in zip(stamps, stamps[1:])]
        print('stamps: %d, deltas ms: %s' % (len(stamps), ', '.join(deltas) or '-'))
    print('phases: %s' % ' > '.join(dict.fromkeys(run['phases'])))
    topics = run['topics']
    if any(topics.values()):
        print('topics received: rgb=%d depth=%d camera_info=%d' % (
            len(topics['rgb']), len(topics['depth']), len(topics['info'])))


# ---------------------------------------------------------------- scenarios

def wait_for_port(port, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            socket.create_connection(('127.0.0.1', port), timeout=0.2).close()
            return True
        except OSError:
            time.sleep(0.1)
    return False


class MockProcess(object):
    def __init__(self, port, extra):
        self.proc = subprocess.Popen(
            [sys.executable, MOCK_PATH, '--port', str(port)] + extra,
            stdout=subprocess.DEVNULL)
        if not wait_for_port(port):
            self.stop()
            raise RuntimeError('mock service did not start on port %d' % port)

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def within_window(stamp, run, tolerance=0.25):
    return run['t_sent'] - tolerance <= to_sec(stamp) <= run['t_done'] + tolerance


def check_rgb_n1(client, args):
    run = client.run_goal('rgb', 1, args.timeout)
    ok = (run['ok'] and run['status'] == GoalStatus.STATUS_SUCCEEDED and run['result'].success
          and bool(run['result'].rgb.data) and not run['result'].depth.data
          and run['result'].camera_info.width > 0 and within_window(run['result'].stamp, run))
    return ok, run


def check_rgbd_n1(client, args):
    run = client.run_goal('rgbd', 1, args.timeout)
    if not run['ok']:
        return False, run
    res = run['result']
    stats = depth_stats(res.depth)
    ok = (run['status'] == GoalStatus.STATUS_SUCCEEDED and res.success
          and res.depth.encoding == '32FC1' and stats is not None
          and 0.0 < stats[0] < 1.0 and 0.1 < stats[1] and stats[2] < 10.0
          and within_window(res.stamp, run))
    return ok, run


def check_rgbd_n5(client, args):
    run = client.run_goal('rgbd', 5, args.timeout)
    if not run['ok']:
        return False, run
    res = run['result']
    stamps = [to_sec(s) for s in res.stamps]
    increasing = all(b > a for a, b in zip(stamps, stamps[1:]))
    topics = run['topics']
    ok = (run['status'] == GoalStatus.STATUS_SUCCEEDED and res.success
          and res.captured_frames == 5 and len(stamps) == 5 and increasing
          and not res.rgb.data and len(topics['rgb']) >= 5 and len(topics['depth']) >= 5
          and len(topics['info']) >= 5)
    return ok, run


def check_clock_skew(client, args):
    run = client.run_goal('rgb', 1, args.timeout)
    ok = (run['ok'] and run['status'] == GoalStatus.STATUS_SUCCEEDED
          and within_window(run['result'].stamp, run)
          and abs(abs(run['result'].clock_offset_ms) - 5000.0) < 500.0)
    return ok, run


def expect_abort(mode, frames):
    def check(client, args):
        run = client.run_goal(mode, frames, args.timeout)
        ok = (run['ok'] and run['status'] == GoalStatus.STATUS_ABORTED
              and not run['result'].success)
        return ok, run
    return check


def check_cancel(client, args):
    run = client.run_goal('rgbd', 50, args.timeout, cancel_after=1.5)
    ok = run['ok'] and run['status'] == GoalStatus.STATUS_CANCELED
    return ok, run


def check_second_goal_rejected(client, args):
    first, info = client.send_goal('rgbd', 50, args.timeout)
    if first is None:
        return False, {'ok': False, 'info': 'first goal: ' + info}
    client.spin_until(lambda: False, 1.0)
    second, second_info = client.send_goal('rgb', 1, args.timeout)
    status, result = client.wait_result(first, 20.0, cancel_after=0.1)
    ok = second is None and second_info == 'goal rejected'
    return ok, {'ok': True, 'status': status, 'result': result, 'phases': [],
                't_sent': 0.0, 't_done': 0.0, 'topics': {}, 'second': second_info}


# name: (check function, mock extra args, needs fault injection)
SCENARIOS = {
    'rgb_n1': (check_rgb_n1, [], False),
    'rgbd_n1': (check_rgbd_n1, [], False),
    'rgbd_n5': (check_rgbd_n5, [], False),
    'clock_skew': (check_clock_skew, ['--clock-skew-s', '5'], True),
    'stale': (expect_abort('rgb', 1), ['--fault', 'stale'], True),
    'latency': (expect_abort('rgb', 1), ['--fault', 'latency', '--latency-ms', '300'], True),
    'drop': (expect_abort('rgbd', 5), ['--fault', 'drop'], True),
    'clock_bad': (expect_abort('rgb', 1), ['--fault', 'clock_bad'], True),
    'cancel': (check_cancel, ['--fps', '5'], False),
    'second_goal_rejected': (check_second_goal_rejected, ['--fps', '5'], False),
}


def run_scenarios(client, args):
    names = args.scenarios or list(SCENARIOS)
    unknown = [n for n in names if n not in SCENARIOS]
    if unknown:
        print('unknown scenarios: %s (known: %s)' % (unknown, ', '.join(SCENARIOS)))
        return 2
    outcomes = []
    for name in names:
        check, extra, faulty = SCENARIOS[name]
        if args.external_mock and (faulty or extra):
            outcomes.append((name, 'SKIP', 'needs its own mock instance'))
            continue
        mock = None if args.external_mock else MockProcess(args.mock_port, extra)
        try:
            ok, run = check(client, args)
        except Exception as exc:  # keep going so one broken scenario does not hide the rest
            ok, run = False, {'ok': False, 'info': 'exception: %s' % exc}
        finally:
            if mock:
                mock.stop()
        detail = ''
        if run.get('ok') and run.get('result') is not None:
            detail = '%s | %s' % (STATUS_NAMES.get(run['status'], run['status']),
                                  run['result'].message)
        elif not run.get('ok'):
            detail = run.get('info', '')
        if 'second' in run:
            detail += ' | second goal: %s' % run['second']
        outcomes.append((name, 'PASS' if ok else 'FAIL', detail))
        print('[%s] %s  %s' % (outcomes[-1][1], name, detail), flush=True)
    failed = [o for o in outcomes if o[1] == 'FAIL']
    print('\n%d passed, %d failed, %d skipped' % (
        sum(o[1] == 'PASS' for o in outcomes), len(failed), sum(o[1] == 'SKIP' for o in outcomes)))
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--action-name', default='/capture_camera_frames')
    parser.add_argument('--mode', choices=['rgb', 'rgbd'], default='rgb')
    parser.add_argument('--num-frames', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=0.0,
                        help='goal timeout_sec (0 = server default)')
    parser.add_argument('--topic-prefix', default=None,
                        help='subscribe to <prefix>{rgb/image_raw,depth/image_raw,camera_info}; '
                             'scenarios default to %s' % DEFAULT_PREFIX)
    parser.add_argument('--scenarios', nargs='*', default=None,
                        help='run the scenario list (all, or the named ones)')
    parser.add_argument('--mock-port', type=int, default=7788,
                        help='port the spawned mock listens on (= server link_port)')
    parser.add_argument('--external-mock', action='store_true',
                        help='scenarios use an already running mock; fault scenarios are skipped')
    args = parser.parse_args()

    rclpy.init()
    prefix = args.topic_prefix
    if args.scenarios is not None and prefix is None:
        prefix = DEFAULT_PREFIX
    client = CaptureClient(args.action_name, prefix)
    try:
        if args.scenarios is not None:
            code = run_scenarios(client, args)
        else:
            run = client.run_goal(args.mode, args.num_frames, args.timeout)
            summarize(run, args.num_frames)
            code = 0 if run['ok'] and run['result'].success else 1
    finally:
        client.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
