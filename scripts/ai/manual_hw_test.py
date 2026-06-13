#!/home/dirk/env/bin/python
"""Manual donkey5 hardware tests.

Examples:
  scripts/manual_hw_test.py stop
  scripts/manual_hw_test.py center
  scripts/manual_hw_test.py steering --angle -1
  scripts/manual_hw_test.py throttle --value 0.15
  scripts/manual_hw_test.py led --mode 0 --seconds 3
  scripts/manual_hw_test.py led --mode -1 --lap --seconds 2
"""

import argparse
import atexit
import signal
import threading
import time

from donkeycar.parts.actuator import PWMThrottle, PWMSteering, PulseController
from donkeycar.parts.led_status import LEDStatusPi
from donkeycar.parts.pins import pwm_pin_by_id
from donkeycar.templates.cfg_donkey5 import (
    STEERING_CHANNEL,
    STEERING_LEFT_PWM,
    STEERING_RIGHT_PWM,
    THROTTLE_CHANNEL,
    THROTTLE_FORWARD_PWM,
    THROTTLE_REVERSE_PWM,
    THROTTLE_STOPPED_PWM,
)


CLEANUPS = []


def run_cleanups():
    while CLEANUPS:
        cleanup = CLEANUPS.pop()
        try:
            cleanup()
        except Exception as exc:
            print(f"Cleanup error: {exc}")


def register_cleanup(cleanup):
    CLEANUPS.append(cleanup)
    return cleanup


def unregister_cleanup(cleanup):
    if cleanup in CLEANUPS:
        CLEANUPS.remove(cleanup)


def on_signal(signum, _frame):
    print(f"Received signal {signum}, running cleanup")
    run_cleanups()
    raise SystemExit(1)


atexit.register(run_cleanups)
signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


class SteeringRig:
    def __init__(self):
        pin = pwm_pin_by_id(STEERING_CHANNEL)
        self.ctrl = PulseController(pin)
        self.part = PWMSteering(
            self.ctrl,
            left_pulse=STEERING_LEFT_PWM,
            right_pulse=STEERING_RIGHT_PWM,
        )

    def set(self, angle: float):
        self.part.run(angle)

    def shutdown(self):
        self.ctrl.shutdown()


class ThrottleRig:
    def __init__(self):
        pin = pwm_pin_by_id(THROTTLE_CHANNEL)
        self.ctrl = PulseController(pin)
        self.part = PWMThrottle(
            self.ctrl,
            max_pulse=THROTTLE_FORWARD_PWM,
            zero_pulse=THROTTLE_STOPPED_PWM,
            min_pulse=THROTTLE_REVERSE_PWM,
        )
        self.thread = threading.Thread(target=self.part.update, daemon=True)
        self.thread.start()

    def set(self, value: float):
        self.part.run(value)

    def stop(self):
        self.part.run(0)

    def shutdown(self):
        self.part.shutdown()


class LedRig:
    def __init__(self):
        self.part = LEDStatusPi()
        self.thread = threading.Thread(target=self.part.update, daemon=True)
        self.thread.start()

    def set(self, mode: int, lap: bool, wipe: bool):
        self.part.run_threaded(mode=mode, lap=lap, wipe=wipe)

    def shutdown(self):
        self.part.shutdown()


def cmd_stop(_args):
    rig = ThrottleRig()
    cleanup = register_cleanup(rig.shutdown)
    try:
        rig.stop()
        print("Throttle stop sent")
    finally:
        unregister_cleanup(cleanup)
        rig.shutdown()


def cmd_center(_args):
    rig = SteeringRig()
    cleanup = register_cleanup(rig.shutdown)
    try:
        rig.set(0)
        print("Steering centered")
    finally:
        unregister_cleanup(cleanup)
        rig.shutdown()


def cmd_steering(args):
    rig = SteeringRig()
    cleanup = register_cleanup(rig.shutdown)
    try:
        rig.set(args.angle)
        print(f"Steering angle sent: {args.angle}")
        time.sleep(args.seconds)
    finally:
        unregister_cleanup(cleanup)
        rig.shutdown()


def cmd_throttle(args):
    rig = ThrottleRig()
    cleanup = register_cleanup(rig.shutdown)
    try:
        rig.set(args.value)
        print(f"Throttle sent: {args.value}")
        time.sleep(args.seconds)
        rig.stop()
        print("Throttle stop sent")
    finally:
        unregister_cleanup(cleanup)
        rig.shutdown()


def cmd_led(args):
    rig = LedRig()
    cleanup = register_cleanup(rig.shutdown)
    try:
        rig.set(args.mode, args.lap, args.wipe)
        print(
            f"LED command sent: mode={args.mode} lap={args.lap} wipe={args.wipe}"
        )
        time.sleep(args.seconds)
    finally:
        unregister_cleanup(cleanup)
        rig.shutdown()


def build_parser():
    parser = argparse.ArgumentParser(description="Manual donkey5 hardware tests")
    sub = parser.add_subparsers(dest="cmd", required=True)

    stop = sub.add_parser("stop", help="Send throttle stop")
    stop.set_defaults(func=cmd_stop)

    center = sub.add_parser("center", help="Center steering")
    center.set_defaults(func=cmd_center)

    steering = sub.add_parser("steering", help="Send one steering angle")
    steering.add_argument("--angle", type=float, required=True)
    steering.add_argument("--seconds", type=float, default=0.5)
    steering.set_defaults(func=cmd_steering)

    throttle = sub.add_parser("throttle", help="Send one throttle value")
    throttle.add_argument("--value", type=float, required=True)
    throttle.add_argument("--seconds", type=float, default=1.0)
    throttle.set_defaults(func=cmd_throttle)

    led = sub.add_parser("led", help="Send one LED command")
    led.add_argument("--mode", type=int, default=-1)
    led.add_argument("--lap", action="store_true")
    led.add_argument("--wipe", action="store_true")
    led.add_argument("--seconds", type=float, default=2.0)
    led.set_defaults(func=cmd_led)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
