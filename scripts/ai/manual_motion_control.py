#!/home/dirk/env/bin/python
"""Safe manual motion control for donkey5."""

import argparse
import atexit
import signal
import time

from donkeycar.parts.actuator import PWMThrottle, PWMSteering, PulseController
from donkeycar.parts.pico import OdometerPico, get_pico, reset_pico
from donkeycar.parts.pins import pwm_pin_by_id
from donkeycar.templates.cfg_donkey5 import (
    ODOMETER_FREQUENCY,
    ODOMETER_GPIO,
    ODOMETER_USE_PIO,
    STEERING_CHANNEL,
    STEERING_LEFT_PWM,
    STEERING_RIGHT_PWM,
    THROTTLE_CHANNEL,
    THROTTLE_FORWARD_PWM,
    THROTTLE_REVERSE_PWM,
    THROTTLE_STOPPED_PWM,
    TICK_PER_M,
)

LOOP_DT = 0.05
MAX_THR = 0.35
STOP_THR = 0.0
BRAKE_ZONE_M = 0.6
DIST_TOL_M = 0.05
ALPHA = 0.35
PULSE_THR = 0.23
PULSE_THR_MAX = 0.27        # max escalation; keep unloaded wheel speed sane
PULSE_THR_STEP = 0.01
PULSE_ON_S = 0.15           # max pulse duration (odometer gates early stop)
PULSE_MIN_ON_S = 0.02       # minimum on-time for ESC to respond
PULSE_READ_DT = 0.010       # odometer poll rate during a pulse
DEBT_TRIGGER_M = 0.02
MOTION_EPS_M = 0.003
READ_ERR_LIMIT = 5
RECOVERY_LIMIT = 1

RIG = None
CENTER_ON_CLEANUP = True


class MotionAbort(RuntimeError):
    pass


class ReadGuard:
    def __init__(self, limit):
        self.limit = limit
        self.errors = 0

    def reset(self):
        self.errors = 0

    def failed(self, exc):
        self.errors += 1
        print(f"read error {self.errors}/{self.limit}: {exc}")
        return self.errors >= self.limit


class RecoveryGuard:
    def __init__(self, limit):
        self.limit = limit
        self.count = 0

    def recover(self):
        if self.count >= self.limit:
            return False
        self.count += 1
        print(f"attempting Pico recovery ({self.count}/{self.limit})...")
        global RIG
        try:
            RIG.stop()
        except Exception:
            pass
        try:
            reset_pico()
            RIG = Rig(center_on_startup=False)
            return True
        except Exception as exc:
            print(f"recovery failed: {exc}")
            return False


class Rig:
    def __init__(self, center_on_startup=True):
        thr_pin = pwm_pin_by_id(THROTTLE_CHANNEL)
        ctrl = PulseController(thr_pin)
        self.throttle = PWMThrottle(
            ctrl,
            max_pulse=THROTTLE_FORWARD_PWM,
            zero_pulse=THROTTLE_STOPPED_PWM,
            min_pulse=THROTTLE_REVERSE_PWM,
        )
        self.odo = OdometerPico(
            pin_id=ODOMETER_GPIO,
            tick_per_meter=TICK_PER_M,
            weight=0.5,
            use_pio=ODOMETER_USE_PIO,
            frequency=ODOMETER_FREQUENCY,
        )
        steer_pin = pwm_pin_by_id(STEERING_CHANNEL)
        steer_ctrl = PulseController(steer_pin)
        self._steering = PWMSteering(
            steer_ctrl,
            left_pulse=STEERING_LEFT_PWM,
            right_pulse=STEERING_RIGHT_PWM,
        )
        if center_on_startup:
            self.center()
        self.stop()

    def center(self):
        try:
            self._steering.run(0)
        except Exception as exc:
            print(f"center command error: {exc}")

    def steer(self, angle):
        try:
            self._steering.run(angle)
        except Exception as exc:
            print(f"steer command error: {exc}")

    def stop(self):
        self.throttle.run(STOP_THR)

    def run_throttle(self, value, limit):
        value = max(STOP_THR, min(limit, value))
        self.throttle.run(value)
        return value

    def read(self):
        return self.odo.run()

    def shutdown(self, center=True):
        self.stop()
        if center:
            try:
                self._steering.run(0)
            except Exception as exc:
                print(f"center cleanup error: {exc}")
        time.sleep(0.2)
        self.odo.shutdown()
        self.throttle.shutdown()


def clamp(val, low, high):
    return max(low, min(high, val))


def create_rig(center_on_startup=True):
    global RIG
    pico = get_pico()
    if pico is None:
        raise MotionAbort("pico not available")
    RIG = Rig(center_on_startup=center_on_startup)
    return RIG


def cleanup():
    global RIG
    if RIG is None:
        return
    try:
        RIG.shutdown(center=CENTER_ON_CLEANUP)
    except Exception as exc:
        print(f"cleanup error: {exc}")
    finally:
        RIG = None


atexit.register(cleanup)


def on_signal(signum, _frame):
    print(f"Received signal {signum}, running cleanup")
    cleanup()
    raise SystemExit(1)


signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


# ---------- command implementations ----------

def cmd_stop(args):
    RIG.stop()
    print("throttle stopped")


def cmd_center(args):
    RIG.center()
    print("steering centered")


def cmd_steering(args):
    RIG.steer(args.angle)
    print(f"steering angle={args.angle:.2f} for {args.seconds:.1f}s")
    time.sleep(args.seconds)
    RIG.center()


def cmd_throttle(args):
    guard = ReadGuard(args.read_error_limit)
    recov = RecoveryGuard(args.recoveries)
    actual = RIG.run_throttle(args.value, args.throttle_limit)
    print(f"throttle={actual:.2f} for {args.seconds:.1f}s")
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        try:
            speed, inst, dist = RIG.read()
            print(
                f"thr={actual:.2f} speed={speed:.3f} "
                f"inst={inst:.3f} dist={dist:.3f}"
            )
            guard.reset()
        except Exception as exc:
            if guard.failed(exc):
                if not recov.recover():
                    raise MotionAbort("too many read errors")
                guard.reset()
                actual = RIG.run_throttle(args.value, args.throttle_limit)
        time.sleep(LOOP_DT)
    RIG.stop()


def cmd_observe(args):
    guard = ReadGuard(args.read_error_limit)
    recov = RecoveryGuard(args.recoveries)
    print(f"observing odometer for {args.seconds:.1f}s")
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        try:
            speed, inst, dist = RIG.read()
            print(f"speed={speed:.3f} inst={inst:.3f} dist={dist:.3f}")
            guard.reset()
        except Exception as exc:
            if guard.failed(exc):
                if not recov.recover():
                    raise MotionAbort("too many read errors")
                guard.reset()
        time.sleep(LOOP_DT)


def cmd_threshold(args):
    guard = ReadGuard(args.read_error_limit)
    recov = RecoveryGuard(args.recoveries)
    val = args.start
    while val <= args.stop + 1e-9:
        clamped = clamp(val, STOP_THR, args.throttle_limit)
        try:
            _, _, d0 = RIG.read()
        except Exception:
            d0 = 0.0
        RIG.run_throttle(clamped, args.throttle_limit)
        t_end = time.time() + args.seconds
        ticks = 0
        moving_ticks = 0
        last_dist = d0
        while time.time() < t_end:
            try:
                speed, inst, dist = RIG.read()
                last_dist = dist
                if speed > 0:
                    moving_ticks += 1
                ticks += 1
                guard.reset()
            except Exception as exc:
                if guard.failed(exc):
                    if not recov.recover():
                        raise MotionAbort("too many read errors")
                    guard.reset()
            time.sleep(LOOP_DT)
        RIG.stop()
        moved = (last_dist - d0) > args.min_distance
        ratio = moving_ticks / max(1, ticks)
        status = "MOTION" if (moved or ratio >= args.min_moving_ratio) else "no motion"
        print(f"throttle={clamped:.2f} {status} moved={last_dist - d0:.3f}m ratio={ratio:.2f}")
        val = round(val + args.step, 4)
        time.sleep(0.5)


def _gated_pulse(rig, pulse_thr, thr_limit, debt_m, pulse_on_s):
    """
    Apply throttle and stop as soon as debt_m distance is covered.
    Falls back to pulse_on_s timeout if the odometer lags.
    Returns actual distance covered during the pulse.
    """
    try:
        _, _, pulse_start = rig.read()
    except Exception:
        pulse_start = None

    rig.run_throttle(pulse_thr, thr_limit)
    t_min = time.time() + PULSE_MIN_ON_S
    t_max = time.time() + pulse_on_s

    while time.time() < t_max:
        time.sleep(PULSE_READ_DT)
        if time.time() < t_min:
            continue
        if pulse_start is None:
            continue
        try:
            _, _, cur = rig.read()
            if cur - pulse_start >= debt_m:
                break
        except Exception:
            pass

    rig.stop()

    if pulse_start is None:
        return 0.0
    try:
        _, _, pulse_end = rig.read()
        return max(0.0, pulse_end - pulse_start)
    except Exception:
        return 0.0


def _run_pulse_loop(rig, target_speed_fn, stop_fn, args, label_fn):
    """
    Generic pulse-control loop.

    target_speed_fn(traveled_m, start_dist) -> target speed (m/s), or None to stop
    stop_fn(traveled_m, start_dist) -> True when distance goal is reached
    label_fn(filtered_speed, inst, traveled_m, pulse_thr, debt) -> status string
    """
    guard = ReadGuard(args.read_error_limit)
    recov = RecoveryGuard(args.recoveries)
    pulse_thr = args.pulse_thr
    pulse_thr_max = args.pulse_thr_max
    pulse_on_s = args.pulse_on_s
    debt = 0.0
    filtered_speed = 0.0
    last_pulse_moved = True  # avoid spurious escalation on first pulse
    try:
        _, _, last_dist = rig.read()
    except Exception:
        last_dist = 0.0
    start_dist = last_dist
    last_t = time.time()

    while True:
        try:
            speed, inst, dist = rig.read()
        except Exception as exc:
            if guard.failed(exc):
                if not recov.recover():
                    raise MotionAbort("too many errors in control loop")
                guard.reset()
                try:
                    _, _, last_dist = rig.read()
                except Exception:
                    pass
            time.sleep(LOOP_DT)
            continue

        guard.reset()
        now = time.time()
        dt = now - last_t
        last_t = now

        delta = dist - last_dist
        last_dist = dist
        traveled = dist - start_dist
        filtered_speed = ALPHA * speed + (1 - ALPHA) * filtered_speed

        if stop_fn(traveled, start_dist):
            rig.stop()
            return

        target = target_speed_fn(traveled, start_dist)
        if target is None:
            rig.stop()
            return

        debt += target * dt - delta

        print(label_fn(filtered_speed, inst, traveled, pulse_thr, debt))

        if debt < DEBT_TRIGGER_M:
            time.sleep(LOOP_DT)
            continue

        # Escalate only when the previous pulse produced no motion
        if not last_pulse_moved:
            pulse_thr = min(pulse_thr_max, pulse_thr + PULSE_THR_STEP)

        covered = _gated_pulse(rig, pulse_thr, args.throttle_limit, debt, pulse_on_s)
        last_pulse_moved = covered >= MOTION_EPS_M
        debt = max(0.0, debt - covered)


def cmd_speed(args):
    t_end = time.time() + args.seconds
    print(f"speed hold target={args.target:.2f} for {args.seconds:.1f}s")

    def target_fn(traveled, start_dist):
        if time.time() >= t_end:
            return None
        return args.target

    def stop_fn(traveled, start_dist):
        return False

    def label_fn(speed, inst, traveled, pulse_thr, debt):
        return (
            f"mode=speed target={args.target:.2f} speed={speed:.3f} "
            f"inst={inst:.3f} traveled={traveled:.3f} "
            f"pulse={pulse_thr:.2f} debt={debt:.3f}"
        )

    _run_pulse_loop(RIG, target_fn, stop_fn, args, label_fn)
    RIG.stop()


def cmd_distance(args):
    guard = ReadGuard(args.read_error_limit)
    recov = RecoveryGuard(args.recoveries)
    min_thr = clamp(args.min_thr, STOP_THR, args.throttle_limit)
    try:
        _, _, start = RIG.read()
    except Exception:
        start = 0.0

    print(f"distance target={args.meters:.2f}m thr={min_thr:.2f}")
    RIG.run_throttle(min_thr, args.throttle_limit)

    while True:
        try:
            speed, inst, dist = RIG.read()
            traveled = dist - start
            remain = args.meters - traveled
            print(
                f"mode=distance target_m={args.meters:.2f} "
                f"traveled={traveled:.3f} remain={remain:.3f} "
                f"speed={speed:.3f} thr={min_thr:.2f}"
            )
            guard.reset()
            if remain <= 0:
                print(f"reached target at traveled={traveled:.3f}m")
                break
        except Exception as exc:
            if guard.failed(exc):
                if not recov.recover():
                    raise MotionAbort("too many read errors")
                guard.reset()
                RIG.run_throttle(min_thr, args.throttle_limit)
        time.sleep(LOOP_DT)

    RIG.stop()


# ---------- argument parser ----------

def add_motion_args(parser):
    parser.add_argument("--throttle-limit", type=float, default=MAX_THR)
    parser.add_argument("--read-error-limit", type=int, default=READ_ERR_LIMIT)
    parser.add_argument("--recoveries", type=int, default=RECOVERY_LIMIT)
    parser.add_argument("--pulse-thr", type=float, default=PULSE_THR)
    parser.add_argument("--pulse-thr-max", type=float, default=PULSE_THR_MAX)
    parser.add_argument("--pulse-on-s", type=float, default=PULSE_ON_S)


def build_parser():
    parser = argparse.ArgumentParser(description="Safe manual motion control")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("stop", help="Send throttle stop")
    p.set_defaults(func=cmd_stop)

    p = sub.add_parser("center", help="Center steering")
    p.set_defaults(func=cmd_center)

    p = sub.add_parser("steering", help="Send one steering angle")
    p.add_argument("--angle", type=float, required=True)
    p.add_argument("--seconds", type=float, default=0.5)
    p.set_defaults(func=cmd_steering)

    p = sub.add_parser("observe", help="Print odometer readings at zero throttle")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--read-error-limit", type=int, default=READ_ERR_LIMIT)
    p.add_argument("--recoveries", type=int, default=RECOVERY_LIMIT)
    p.set_defaults(func=cmd_observe)

    p = sub.add_parser("throttle", help="Apply a fixed throttle value")
    p.add_argument("--value", type=float, required=True)
    p.add_argument("--seconds", type=float, default=1.0)
    add_motion_args(p)
    p.set_defaults(func=cmd_throttle)

    p = sub.add_parser("threshold", help="Sweep throttle to find motion threshold")
    p.add_argument("--start", type=float, default=0.20)
    p.add_argument("--stop", type=float, default=0.35)
    p.add_argument("--step", type=float, default=0.02)
    p.add_argument("--seconds", type=float, default=0.8)
    p.add_argument("--min-distance", type=float, default=0.15)
    p.add_argument("--min-moving-ratio", type=float, default=0.25)
    add_motion_args(p)
    p.set_defaults(func=cmd_threshold)

    p = sub.add_parser("speed", help="Hold a target speed")
    p.add_argument("--target", type=float, required=True)
    p.add_argument("--seconds", type=float, default=5.0)
    add_motion_args(p)
    p.set_defaults(func=cmd_speed)

    p = sub.add_parser("distance", help="Drive a set distance at continuous min throttle")
    p.add_argument("--meters", type=float, required=True)
    p.add_argument("--min-thr", type=float, default=PULSE_THR)
    p.add_argument("--throttle-limit", type=float, default=MAX_THR)
    p.add_argument("--read-error-limit", type=int, default=READ_ERR_LIMIT)
    p.add_argument("--recoveries", type=int, default=RECOVERY_LIMIT)
    p.set_defaults(func=cmd_distance)

    return parser


def validate_args(args):
    if hasattr(args, "throttle_limit"):
        args.throttle_limit = clamp(args.throttle_limit, 0.0, MAX_THR)
    if getattr(args, "read_error_limit", 1) < 1:
        raise SystemExit("--read-error-limit must be >= 1")
    if getattr(args, "recoveries", 0) < 0:
        raise SystemExit("--recoveries must be >= 0")
    if hasattr(args, "step") and args.step <= 0:
        raise SystemExit("--step must be > 0")
    if hasattr(args, "stop") and hasattr(args, "start"):
        if args.stop < args.start:
            raise SystemExit("--stop must be >= --start")


def use_steering(args):
    return args.cmd not in {"observe", "throttle", "threshold", "distance"}


def set_cleanup_mode(args):
    global CENTER_ON_CLEANUP
    CENTER_ON_CLEANUP = use_steering(args)


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_args(args)
    set_cleanup_mode(args)

    create_rig(center_on_startup=use_steering(args))

    try:
        args.func(args)
    except MotionAbort as exc:
        print(f"abort: {exc}")
        raise SystemExit(1)
    finally:
        cleanup()


if __name__ == "__main__":
    main()
