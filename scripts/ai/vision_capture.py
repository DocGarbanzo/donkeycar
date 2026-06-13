#!/home/dirk/env/bin/python
"""Camera capture and TFLite inference harness for donkey5."""

import argparse
import atexit
import signal
import threading
import time

import cv2
import numpy as np

from donkeycar.parts.camera import PiCamera
from donkeycar.templates.cfg_donkey5 import IMAGE_H, IMAGE_W, IMAGE_DEPTH

DEFAULT_MODEL = "/home/dirk/mycar/models/pilot_25-08-31_9.tflite"
LOOP_DT = 0.05   # 20 Hz control cycle

CAM = None
CAM_THREAD = None


def open_camera():
    global CAM, CAM_THREAD
    cam = PiCamera(image_w=IMAGE_W, image_h=IMAGE_H, image_d=IMAGE_DEPTH)
    t = threading.Thread(target=cam.update, daemon=True)
    t.start()
    CAM = cam
    CAM_THREAD = t
    return cam


def close_camera():
    global CAM
    if CAM is not None:
        try:
            CAM.shutdown()
        except Exception as exc:
            print(f"camera shutdown error: {exc}")
        CAM = None


atexit.register(close_camera)


def on_signal(signum, _frame):
    print(f"signal {signum} received, shutting down")
    close_camera()
    raise SystemExit(1)


signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


# ---------- observe command ----------

def cmd_observe(args):
    cam = open_camera()
    time.sleep(0.5)   # let auto-exposure settle
    print(f"observing for {args.seconds:.0f}s  (saving snapshots to /tmp/)")
    t_end = time.time() + args.seconds
    snapshot_i = 0
    while time.time() < t_end:
        frame = cam.run_threaded()
        if frame is None:
            time.sleep(LOOP_DT)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi = gray[IMAGE_H // 2:, :]           # bottom half
        edges = cv2.Canny(roi, 50, 150)

        brightness = float(gray.mean())
        roi_brightness = float(roi.mean())
        edge_density = float(edges.mean())
        bgr_means = frame.mean(axis=(0, 1))

        print(
            f"brightness={brightness:.1f} roi={roi_brightness:.1f} "
            f"edges={edge_density:.2f} "
            f"B={bgr_means[0]:.0f} G={bgr_means[1]:.0f} R={bgr_means[2]:.0f}"
        )

        if args.snapshot_interval > 0:
            snapshot_i += 1
            if snapshot_i % max(1, int(args.snapshot_interval / LOOP_DT)) == 0:
                path = f"/tmp/vision_{int(time.time())}.jpg"
                cv2.imwrite(path, frame)
                print(f"  saved {path}")

        time.sleep(LOOP_DT)


# ---------- infer command ----------

def load_interpreter(model_path):
    from ai_edge_litert.interpreter import Interpreter
    interp = Interpreter(model_path=model_path)
    interp.allocate_tensors()
    inp = interp.get_input_details()
    out = interp.get_output_details()
    return interp, inp, out


def run_inference(interp, inp, out, frame):
    f32 = frame.astype(np.float32) / 255.0
    interp.set_tensor(inp[0]['index'], f32[np.newaxis])
    interp.invoke()
    angle    = float(interp.get_tensor(out[0]['index']).flat[0])
    throttle = float(interp.get_tensor(out[1]['index']).flat[0])
    return angle, throttle


def cmd_infer(args):
    print(f"loading model: {args.model}")
    interp, inp, out = load_interpreter(args.model)
    print(f"input shape: {inp[0]['shape']}  output shapes: {[o['shape'] for o in out]}")

    cam = open_camera()
    time.sleep(0.5)
    print(f"running inference at ~{1/LOOP_DT:.0f} Hz for {args.seconds:.0f}s")
    t_end = time.time() + args.seconds
    latencies = []
    while time.time() < t_end:
        frame = cam.run_threaded()
        if frame is None:
            time.sleep(LOOP_DT)
            continue
        t0 = time.time()
        angle, throttle = run_inference(interp, inp, out, frame)
        lat_ms = (time.time() - t0) * 1000
        latencies.append(lat_ms)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = float(gray.mean())
        print(
            f"angle={angle:+.3f}  throttle={throttle:.3f}  "
            f"brightness={brightness:.0f}  latency={lat_ms:.2f}ms"
        )
        time.sleep(LOOP_DT)

    if latencies:
        print(
            f"\nlatency: mean={np.mean(latencies):.2f}ms "
            f"p95={np.percentile(latencies, 95):.2f}ms "
            f"max={max(latencies):.2f}ms"
        )


# ---------- argument parser ----------

def build_parser():
    parser = argparse.ArgumentParser(description="Camera and vision harness")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("observe", help="Log brightness and edge stats, save snapshots")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--snapshot-interval", type=float, default=2.0,
                   help="seconds between JPEG snapshots (0 = none)")
    p.set_defaults(func=cmd_observe)

    p = sub.add_parser("infer", help="Run TFLite pilot inference on live frames")
    p.add_argument("--model", type=str, default=DEFAULT_MODEL)
    p.add_argument("--seconds", type=float, default=10.0)
    p.set_defaults(func=cmd_infer)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.func(args)
    finally:
        close_camera()


if __name__ == "__main__":
    main()
