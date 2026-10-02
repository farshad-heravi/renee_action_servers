# Camera link protocol (PC ⇄ Jetson), version 1

Contract between the PC-side `capture_camera_frames_action_server` and the camera
service that runs next to the camera (the Jetson with the ZED2i). The Jetson
implements the **server** side of this document; the PC node is the **client**.
`scripts/mock_camera_service.py` is a reference implementation of the server.

```mermaid
sequenceDiagram
    participant PC as PC action server
    participant J as Jetson camera service
    PC->>J: ping {t0}
    J-->>PC: pong {t0, t1, t2, clock_ok, chrony_offset_ms}
    Note over PC: offset, rtt from t0..t3 (min-RTT of several pings)
    PC->>J: capture {mode, num_frames, t0}
    loop N frames
        J-->>PC: frame {index, capture_ts_ns, rgb[, depth], camera_info} + blobs
    end
    J-->>PC: done {n_frames}
```

## 1. Transport and framing

- One TCP connection, kept open between goals. The PC reconnects on its own if it
  drops. The Jetson must accept a new connection at any time. How the PC reaches
  the port (SSH tunnel or direct route) is deployment config, not part of the protocol.
- Every message, in both directions:

  | bytes | content |
  |---|---|
  | 4 | `uint32` **big-endian**: length `H` of the JSON header |
  | `H` | UTF-8 JSON object (the header) |
  | `sum(blob_sizes)` | binary blobs, concatenated in the order of `blob_sizes` |

- Every header has `"version": 1`, a `"type"` string and `"blob_sizes"` (list of
  byte counts, `[]` if the message has no blobs). A receiver that sees another
  `version` replies with an `error` message.
- All timestamps are `int64` **nanoseconds since the Unix epoch** in the sender's own
  clock (JSON integers, no floats).

## 2. Messages

### Client → server

| type | fields | meaning |
|---|---|---|
| `ping` | `t0` (PC clock, ns) | clock-sync probe |
| `capture` | `mode` (`"rgb"` or `"rgbd"`), `num_frames` (≥ 1), `t0` (PC clock, ns) | grab `num_frames` consecutive frames |
| `cancel` | – | stop an ongoing capture |

### Server → client

| type | fields | meaning |
|---|---|---|
| `pong` | `t0` (echoed), `t1` (Jetson clock when the ping was received), `t2` (Jetson clock just before sending), `clock_ok` (bool), `chrony_offset_ms` (float or `null`) | reply to `ping` |
| `frame` | see below | one captured frame, followed by its blobs |
| `done` | `n_frames` (frames actually sent), optional `cancelled: true` | end of a `capture` |
| `error` | `message` | request rejected or capture failed; ends the capture |

`frame` header:

```json
{
  "version": 1, "type": "frame", "index": 0,
  "capture_ts_ns": 1760000000123456789,
  "rgb":   {"width": 1280, "height": 720, "encoding": "bgr8",  "codec": "zlib"},
  "depth": {"width": 1280, "height": 720, "encoding": "16UC1", "codec": "zlib"},
  "camera_info": {"width": 1280, "height": 720, "distortion_model": "plumb_bob",
                  "D": [k1, k2, t1, t2, k3], "K": [9 numbers], "R": [9 numbers], "P": [12 numbers]},
  "blob_sizes": [rgb_compressed_bytes, depth_compressed_bytes]
}
```

- `depth` is absent in `rgb` mode, and `blob_sizes` then has one entry.
- `camera_info` is sent with **every** frame (the PC does not have to cache it).
  `K`, `R`, `P` are row-major, same layout as `sensor_msgs/CameraInfo`.
  The intrinsics are those of the **left** camera at the streamed resolution;
  depth is registered to the left image, so it shares them.
- `index` counts from 0 within one `capture`. Indices are contiguous unless a frame
  was lost; the PC treats a gap as an error.
- `capture_ts_ns` is the exposure/capture time of **this** frame in the Jetson
  clock (for the ZED SDK: `get_timestamp(TIME_REFERENCE.IMAGE)`), not the time the
  frame was encoded or sent. RGB and depth of one frame come from the same
  `grab()` and share this single timestamp.

## 3. Pixel formats

Blobs are the **raw row-major pixel buffer, no row padding**, compressed with zlib
(`zlib.compress(buf, 1)`; level 1 keeps the Nano's CPU use low).

| stream | decompressed layout | size |
|---|---|---|
| `rgb` | `bgr8`: 3 × `uint8` per pixel, B,G,R order | `w*h*3` |
| `depth` | `16UC1`: 1 × `uint16` little-endian, **millimetres** along the optical axis, **0 = invalid** (NaN/inf/out of range in the SDK) | `w*h*2` |

The PC converts depth to `32FC1` metres (0 → NaN) before publishing. The `codec`
field exists so another codec (e.g. `png`) can be added later without a version bump;
the PC rejects codecs it does not know.

## 4. Timing and clock synchronisation

The PC measures the Jetson clock offset itself, so correctness does not depend on
chrony being healthy:

```
PC sends ping at t0, Jetson receives at t1, replies at t2, PC receives at t3 (PC clock)
offset = ((t1 - t0) + (t2 - t3)) / 2        # Jetson clock minus PC clock
rtt    = (t3 - t0) - (t2 - t1)
PC time of a frame = capture_ts_ns - offset
```

The PC sends several pings and keeps the sample with the smallest `rtt`; it also
re-measures periodically while idle. Therefore:

- Take `t1` as soon as the message is read and `t2` as late as possible before sending
  (no processing between them).
- Answer `ping` promptly even while a capture runs.
- `clock_ok` is `false` until the Jetson clock has been synchronised since boot
  (the Nano has no battery RTC, so its clock is wrong after every boot); the PC
  refuses to capture while it is `false`. `chrony_offset_ms` is the current chrony
  estimate (`chronyc tracking`), or `null` if unavailable. Both are only a sanity check on top of
  the measured offset.

## 5. Jetson-side responsibilities

1. **Warm camera.** Open the ZED once at service start and keep it open, with a
   background grab loop (e.g. 10-15 fps) so auto-exposure and the image buffer stay
   fresh. Warm-up happens once, not per request.
2. **Depth only on request.** The idle loop grabs without depth (`enable_depth=False`
   in `RuntimeParameters`, if the SDK version supports it). Compute depth only for
   `rgbd` captures.
3. **Only fresh frames.** A frame may be returned only if it was captured **after the
   `capture` message was received** (`capture_ts_ns` ≥ the Jetson's own receive time).
   Discard everything buffered earlier. The request `t0` is in the PC clock and is
   informational only; do not compare it with Jetson timestamps.
4. **Consecutive frames.** For `num_frames = N` send N distinct, strictly increasing
   frames as they are grabbed, each in its own `frame` message (the PC publishes them
   as they arrive), then `done`. No files are written.
5. **Cancel.** On `cancel`, stop after the current frame and send
   `done` with the number of frames sent and `"cancelled": true`.
6. **Errors.** Camera not available, bad `mode`, `num_frames` < 1, unknown message type or wrong
   `version`: send `error` and keep the connection open. A capture that cannot complete
   ends with `error` instead of `done`.
7. **One camera owner.** The service is the only process that opens the ZED; other
   tools must go through it.
8. **Clock.** Run chrony on the Jetson, synchronised to the main board (which is
   synchronised to the PC). Report its state in `pong`.
9. **Python 3.6 / stdlib.** The Nano has no internet and Python 3.6: use only the
   standard library (`socket`, `struct`, `json`, `zlib`, `select`, `threading`) plus the
   already installed ZED SDK and numpy. No f-string-only or 3.7+ APIs (e.g. `time.time_ns`).

## 6. What the PC checks (for reference)

round trip ≤ `max_rtt_ms`; `clock_ok` true (and `chrony_offset_ms` small if present);
every converted capture time ≥ goal-received time − `rtt/2` and not in the future;
strictly increasing timestamps; contiguous `index`; `camera_info` size equals image size;
RGB and depth sizes consistent. Any failure aborts the goal with a reason.

## 7. Example (rgb, one frame)

```
PC  → {"version":1,"type":"ping","t0":1000,"blob_sizes":[]}
J   → {"version":1,"type":"pong","t0":1000,"t1":5200,"t2":5210,"clock_ok":true,"chrony_offset_ms":0.4,"blob_sizes":[]}
PC  → {"version":1,"type":"capture","mode":"rgb","num_frames":1,"t0":2000,"blob_sizes":[]}
J   → {"version":1,"type":"frame","index":0,"capture_ts_ns":...,"rgb":{...},"camera_info":{...},"blob_sizes":[812345]} + 812345 bytes
J   → {"version":1,"type":"done","n_frames":1,"blob_sizes":[]}
```

## 8. Testing without the camera

`scripts/mock_camera_service.py` serves this protocol with synthetic frames and
fault injection (`--clock-skew-s`, `--fault stale|latency|drop|clock_bad`).
`scripts/test_capture_client.py` drives the PC action and can run the whole scenario
list against the mock.

## 9. Versioning

`version` is 1. Adding optional fields keeps version 1; changing framing, units or
required fields bumps it. Unknown extra fields must be ignored by both sides.
