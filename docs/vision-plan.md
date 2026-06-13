# Vision plan for donkey5

This document describes how to capture, process, and interpret camera
images on this hardware, and how to combine vision with the motion
control stack from `docs/manual-motion-control-plan.md`.

## Hardware confirmed

- **Camera**: OV5647 (Raspberry Pi Camera Module v1), CSI connector
- **ISP**: Raspberry Pi 5 PiSP backend (`pispbe`) handles auto-exposure
  and white-balance in hardware
- **Interface**: `rp1-cfe` kernel driver, accessed via `libcamera` /
  `picamera2`

## Library availability

| Library | Available | Notes |
|---------|-----------|-------|
| `picamera2` | yes | wraps `libcamera`; what donkey5 uses |
| `opencv 4.13.0` | yes | full build including DNN module |
| `ai_edge_litert` | yes | Google's TFLite successor; XNNPACK delegate active |
| `PIL` | yes | |
| `tensorflow` | no | not installed; not needed |
| `torch` | no | not installed |

## Measured performance (at 192 × 144 px)

All numbers measured on this Pi 5 with the OV5647.

### Capture

| Mode | FPS |
|------|-----|
| Default (no frame limit) | ~19 fps |
| With `FrameDurationLimits=(25000,25000)` | **40.5 fps** |

40 fps matches `DRIVE_LOOP_HZ` in `cfg_donkey5.py`.  The donkeycar
`PiCamera` part applies this limit automatically via the camera
warm-up configuration.

### OpenCV operations (per frame, 192 × 144)

| Operation | ms/frame | Equivalent fps |
|-----------|----------|----------------|
| Canny edge detection | 0.18 ms | 5400 |
| GaussianBlur 5×5 | 0.03 ms | 33000 |
| BGR → HSV | 0.15 ms | 6600 |
| NumPy mean / brightness | 0.48 ms | 2100 |

All operations are well below the 25 ms budget per frame.

### TFLite inference

| Model | Input shape | ms/frame | Equivalent fps |
|-------|-------------|----------|----------------|
| pilot (KerasSquarePlus) | (1, 144, 192, 3) | **1.64 ms** | 611 |

Inference is fast enough to run every frame at 40 fps with plenty
of headroom.

## Camera output format

- `picamera2` is configured with `format="BGR888"` but the donkeycar
  docs note it returns RGB.  The `PiCamera` part returns a numpy array
  of shape `(144, 192, 3)`, dtype `uint8`.
- When passing to TFLite: normalize to `float32` in `[0, 1]`
  (`frame.astype(np.float32) / 255.0`).
- When passing to OpenCV: frame is treated as BGR unless explicitly
  converted.

## Donkeycar PiCamera part (threaded)

```python
from donkeycar.parts.camera import PiCamera
import threading

cam = PiCamera(image_w=192, image_h=144, image_d=3)
t = threading.Thread(target=cam.update, daemon=True)
t.start()
# cam.run_threaded() returns the latest cached frame (instant)
frame = cam.run_threaded()   # numpy (144, 192, 3) uint8
cam.shutdown()
```

`run_threaded()` is instant (returns cached frame).  The background
thread runs at ~40 fps and updates `cam.frame` continuously.

## Existing TFLite models

Trained donkey5 pilot models already exist on this Pi:

```
/home/dirk/mycar/models/pilot_25-08-31_9.tflite   ← most recent
/home/dirk/mycar/models/pilot_25-08-31_8.tflite
/home/dirk/mycar/models/pilot_25-07-28_4.tflite
```

All take `(1, 144, 192, 3) float32` input and produce two `(1, 1)`
float32 tensors: `[angle, throttle]`.

Running inference:

```python
from ai_edge_litert.interpreter import Interpreter
import numpy as np

interp = Interpreter(model_path="/home/dirk/mycar/models/pilot_25-08-31_9.tflite")
interp.allocate_tensors()
inp = interp.get_input_details()
out = interp.get_output_details()

frame_f32 = frame.astype(np.float32) / 255.0
interp.set_tensor(inp[0]['index'], frame_f32[np.newaxis])
interp.invoke()
angle    = float(interp.get_tensor(out[0]['index']))
throttle = float(interp.get_tensor(out[1]['index']))
```

## Possible vision tasks (in order of complexity)

### 1. Passive observation — capture + log frames

Just capture frames and log brightness, colour histograms, and a saved
JPEG periodically.  Useful for checking framing, exposure, and
orientation before doing anything smarter.

Script: `scripts/ai/vision_capture.py observe`

### 2. Inline OpenCV analysis — no ML

At 40 fps, all of the following fit in the 25 ms budget:

- **Brightness**: mean pixel value in a region of interest (ROI)
- **Lane edge detection**: Canny on the bottom half of the frame
- **Colour mask**: HSV range mask to isolate road vs. not-road
- **Horizon estimation**: find the horizontal band with the most
  edge density → upper bound for "road" region

These are cheap and deterministic — useful for sanity-checking
the image content and as inputs to a hand-tuned controller.

### 3. TFLite pilot inference — live angle + throttle

Run the existing trained model on every captured frame.
Output is `(angle, throttle)` that can be fed directly to the
motion stack.

Latency budget at 40 fps:
- Capture (cached, threaded): ~0 ms
- BGR→float32 normalize: ~0.1 ms
- TFLite invoke: ~1.64 ms
- Total: **< 2 ms** — leaves 23 ms for everything else

### 4. Combined vision + motion loop

See section below.

## Integrating vision with motion

### Architecture

```
┌──────────────┐  frame (144×192×3)   ┌─────────────────┐
│  PiCamera    │ ──────────────────→  │  VisionWorker   │
│  (threaded)  │                      │  (threaded)      │
└──────────────┘                      │                  │
                                      │  OpenCV / TFLite │
                                      │  → angle, flags  │
                                      └────────┬─────────┘
                                               │ angle, stop_flag
                                      ┌────────▼─────────┐
                                      │  MotionLoop      │
                                      │  (main thread)   │
                                      │                  │
                                      │  odometer read   │
                                      │  throttle write  │
                                      │  steering write  │
                                      └──────────────────┘
```

The vision worker runs in a background thread (like the camera).
It caches the latest `(angle, stop_flag, distance_remaining)` tuple.
The motion loop reads this cache each cycle and acts on it.

### Timing

- Camera background thread: 40 fps, updates `cam.frame`
- Vision worker: reads `cam.frame`, runs inference (~2 ms), writes
  result; can run at 40 fps if needed, or slower if desired
- Motion loop: 20 Hz (50 ms cycle), reads cached vision result,
  sends throttle and steering

At 20 Hz the motion loop has 50 ms.  Vision inference (2 ms) and
OpenCV analysis (< 1 ms) together take 3 ms — 47 ms of headroom.

### Steering control

For motion, the `Rig` class already supports `steer(angle)`.
For vision-guided steering:

```python
# angle from TFLite: typically in [-1, 1]
# clamp and smooth before sending to avoid jerk
smoothed_angle = 0.7 * prev_angle + 0.3 * new_angle
RIG.steer(clamp(smoothed_angle, -1.0, 1.0))
```

### Safety rules (inherited from motion plan)

1. Throttle must be set to zero before exit on every path
2. Steering should be centered on clean exit
3. If vision inference fails or stalls, stop immediately
4. No overlapping Pico sessions

## Trial-and-error results (2026-06-07)

### Stage 1: camera harness — DONE

- `PiCamera` part initialises cleanly, captures at **40.5 fps** with
  `FrameDurationLimits=(25000,25000)`
- OpenCV Canny: **0.18 ms**, GaussianBlur: **0.03 ms**,
  NumPy brightness: **0.48 ms** — all negligible at 192×144
- TFLite pilot inference: **mean 2.31 ms, p95 3.14 ms** per frame
- `scripts/ai/vision_capture.py` implemented with `observe` and
  `infer` commands — both verified working

### `observe` output (static scene, indoor)

```
brightness=85.9 roi=59.1 edges=24.3 B=74 G=78 R=105
```

Very stable — auto-exposure settled within 0.5 s.

### `infer` output (static scene, existing pilot model)

```
angle=-0.91  throttle=0.22  brightness=84  latency=2.31ms
```

The model produces a strongly negative angle (~−0.91) on the current
static scene (camera probably pointing somewhere off-track).  The
output is stable across frames.  This is expected — the model only
makes sense when the car is on the track it was trained on.

**Important**: the angle and throttle from the existing pilot reflect
what the model was trained on (the mycar track).  The output will
only be meaningful for vision-guided driving when the car is in a
recognisable track context.

### Stage 2: vision observe script

Create `scripts/ai/vision_capture.py` with an `observe` command.

Behaviour:
- open camera in threaded mode
- every 0.5 s, log brightness, mean BGR, ROI edge count
- save a JPEG snapshot to `/tmp/` for inspection
- run for `--seconds N` then exit cleanly

This confirms framing and exposure without any risk of motion.

### Stage 3: inline TFLite inference

Add an `infer` command to `vision_capture.py`.

Behaviour:
- run at 20 Hz
- each cycle: normalize frame, invoke TFLite, print angle + throttle
- print latency per cycle
- no motion output yet

This confirms the model produces sensible angle values on live images.

### Stage 4: vision-guided distance drive

Extend `manual_motion_control.py distance` to optionally accept
steering from a vision thread.

Behaviour:
- continuous minimum throttle (existing approach — confirmed good)
- steering set each cycle from TFLite angle output
- smooth angle with exponential filter to prevent jerk
- stop at target odometer distance (existing approach)

Command:
```bash
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py \
    distance --meters 5 --steer-from-vision
```

### Stage 5: open-loop vision drive (no distance target)

Drive at constant throttle, steering from vision, until Ctrl+C or
a stop condition (e.g., brightness drops below threshold = obstacle /
end of track).

```bash
... manual_motion_control.py speed --target 0.3 --steer-from-vision
```

## Known constraints

1. **OV5647 is the v1 camera** — fixed-focus, lower dynamic range than
   v2 or HQ camera.  Works, but may struggle in low light.
2. **No trained object-detection model** yet — only a steering/throttle
   pilot.  Object detection would require installing a model separately.
3. **Pico serial errors** can occur on long runs — the motion code
   already handles recovery; vision loop must tolerate pauses.
4. **No display on the Pi** — use `cv2.imwrite` to save frames to
   `/tmp/` and copy them off for inspection.

## Script location

All vision scripts live in `scripts/ai/`.

- `scripts/ai/vision_capture.py` — camera and inference harness
- `scripts/ai/manual_motion_control.py` — motion harness (will gain
  `--steer-from-vision` flag in Stage 4)

## Recommended next steps

1. Run `scripts/ai/vision_capture.py observe` and inspect saved frames
2. Run `scripts/ai/vision_capture.py infer` and check that the model
   produces stable angle estimates on real images
3. Once angle looks sensible, attempt Stage 4 on a short distance run
