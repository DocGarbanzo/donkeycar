# donkey5 hardware control notes

This document captures manual hardware testing notes for the
`donkeycar/templates/donkey5.py` setup on this repo.

## Environment

- Repo: `~/projects/donkeycar`
- Branch used during testing: `dev`
- Python env used during testing: `~/env`
- Safe interpreter path: `/home/dirk/env/bin/python`

Use the env Python explicitly when doing manual hardware tests.

## Relevant donkey5 parts

### LED

`donkey5.py` does not use `RGB_LED`.
It uses:

- `donkeycar.parts.led_status.LEDStatusPi`

This part wraps `gpiozero.RGBLED` and is intended to run as a threaded
part:

- `update()` runs in a background thread
- `run_threaded(mode, lap, wipe)` updates state and queues events

Default pins:

- red: `GPIO6`
- green: `GPIO13`
- blue: `GPIO19`

These are BCM-style gpiozero pin names.

### Steering

Steering path in `donkey5.py`:

- `PWMSteering`
- `PulseController`
- `PwmPinPico`
- Pico USB comms

From the vehicle perspective, steering accepts a normalized angle in
`[-1, 1]`.

Config from `cfg_donkey5.py`:

- `STEERING_CHANNEL = "PICO.BCM.16"`
- `STEERING_LEFT_PWM = 220`
- `STEERING_RIGHT_PWM = 500`

Useful manual command:

```python
steering.run(0)
```

This centers the wheels.

### Throttle

Throttle path in `donkey5.py`:

- `PWMThrottle`
- `PulseController`
- `PwmPinPico`
- Pico USB comms

Config from `cfg_donkey5.py`:

- `THROTTLE_CHANNEL = "PICO.BCM.17"`
- `THROTTLE_FORWARD_PWM = 460`
- `THROTTLE_STOPPED_PWM = 370`
- `THROTTLE_REVERSE_PWM = 280`

Important: the stopped pulse matters. Sending a command and then just
killing the interpreter is not enough to guarantee a safe stop.

### Odometer

`donkey5.py` uses:

- `donkeycar.parts.pico.OdometerPico`

Config from `cfg_donkey5.py`:

- `ODOMETER_GPIO = "PICO.BCM.2"`
- `TICK_PER_M = 75`
- `ODOMETER_USE_PIO = True`
- `ODOMETER_FREQUENCY = 20000`

## LED behavior confirmed during testing

Tested in a live Python session with a manual background thread for
`led.update()`.

Exact setup snippet:

```python
from donkeycar.parts.led_status import LEDStatusPi
import threading

led = LEDStatusPi()
led_thread = threading.Thread(target=led.update, daemon=True)
led_thread.start()
```

Confirmed behavior:

- `mode=0` -> continuous green blink
- `mode=1` -> continuous yellow blink
- `lap=True` -> red blink sequence
- `wipe=True` -> blue blink sequence
- repeated lap/wipe events queue correctly

Exact test snippets:

```python
led.run_threaded(mode=0, lap=False, wipe=False)
led.run_threaded(mode=1, lap=False, wipe=False)
led.run_threaded(mode=-1, lap=False, wipe=False)
led.run_threaded(mode=-1, lap=True, wipe=False)
led.run_threaded(mode=-1, lap=False, wipe=True)
```

Queueing test:

```python
import time
led.run_threaded(mode=-1, lap=True, wipe=False)
time.sleep(0.1)
led.run_threaded(mode=-1, lap=False, wipe=True)
time.sleep(0.1)
led.run_threaded(mode=-1, lap=True, wipe=False)
time.sleep(0.1)
led.run_threaded(mode=-1, lap=False, wipe=True)
```

### Off mode extension

`LEDStatusPi` originally had no off mode for continuous blinking.
A local code change was made so that:

- `mode=-1` -> LED off

Implementation lives in:

- `donkeycar/parts/led_status.py`

The continuous blink is driven by `gpiozero` in background mode via:

```python
self.led.blink(..., background=True)
```

So the part has:

- one Donkey background thread for `update()`
- gpiozero-managed blinking for the continuous state

## Manual testing observations

### Steering

Manual steering through `PWMSteering` works.

Exact setup snippet:

```python
from donkeycar.templates.cfg_donkey5 import (
    STEERING_CHANNEL, STEERING_LEFT_PWM, STEERING_RIGHT_PWM,
)
from donkeycar.parts.pins import pwm_pin_by_id
from donkeycar.parts.actuator import PulseController, PWMSteering

steering_pin = pwm_pin_by_id(STEERING_CHANNEL)
steering_ctrl = PulseController(steering_pin)
steering = PWMSteering(
    steering_ctrl,
    left_pulse=STEERING_LEFT_PWM,
    right_pulse=STEERING_RIGHT_PWM,
)
```

Tested commands included:

```python
steering.run(-1)
steering.run(0)
steering.run(1)
```

A smooth sweep is possible by sending many small intermediate steering
values.

Exact sweep used during testing:

```python
import time
angles = [
    i / 100
    for i in list(range(0, -101, -1))
    + list(range(-99, 101, 1))
    + list(range(99, -101, -1))
    + list(range(-99, 1, 1))
]
[steering.run(a) or time.sleep(0.005) for a in angles]
```

### Threaded vs non-threaded steering

Both approaches worked similarly in live testing:

- `steering.run(value)`
- `steering.run_threaded(value)` with `steering.update()` running in a
  background thread

For the tested sweep there was no visible difference.

### Throttle / speed control

Exact setup snippet:

```python
from donkeycar.templates.cfg_donkey5 import (
    THROTTLE_CHANNEL, THROTTLE_FORWARD_PWM,
    THROTTLE_STOPPED_PWM, THROTTLE_REVERSE_PWM,
)
from donkeycar.parts.pins import pwm_pin_by_id
from donkeycar.parts.actuator import PulseController, PWMThrottle
import threading

throttle_pin = pwm_pin_by_id(THROTTLE_CHANNEL)
throttle_ctrl = PulseController(throttle_pin)
throttle = PWMThrottle(
    throttle_ctrl,
    max_pulse=THROTTLE_FORWARD_PWM,
    zero_pulse=THROTTLE_STOPPED_PWM,
    min_pulse=THROTTLE_REVERSE_PWM,
)
throttle_thread = threading.Thread(target=throttle.update, daemon=True)
throttle_thread.start()
```

Useful manual commands:

```python
throttle.run(0.15)
throttle.run(0)
```

Ad-hoc closed-loop speed experiments using `PWMThrottle` and
`OdometerPico` were unstable and noisy.

Observed issues from early ad-hoc control attempts:

- bursty odometer readings
- oscillating throttle commands
- hard-to-control live REPL session once a print loop was running

Bench threshold findings with wheels in the air:

- `0.10` to `0.12` did not produce sustained motion
- `0.13` crossed the motion threshold
- `0.15` already produced very high unloaded wheel speed

This suggests motion control should use short pulses above the threshold
rather than trying to hold a tiny continuous throttle.

Additional bench findings:

- `speed --target 0.3` worked with a threshold-based pulse controller
- pulse-based `distance` runs were visually stuttery and caused a
  steering jerk on startup — that approach was abandoned
- continuous-throttle `distance --meters 10` runs completed smoothly
  with ~1–8 cm overshoot; confirmed good by user
- correct approach: hold minimum throttle (`0.23`) continuously, stop
  the moment odometer reports target distance reached; no steering
  commands sent during distance runs

On-ground threshold findings (wheels on floor, car at rest):

- `0.15` produced no motion
- `0.16` marginal — stalled twice before getting moving, not reliable
- `0.17` reliable minimum — gets moving within ~4 loops, no stalls
- `0.18` and above: clean reliable motion
- **confirmed on-ground minimum: `0.17`**

The on-ground threshold is higher than the unloaded bench threshold due
to added friction and vehicle weight.

## Safe manual testing procedure

Safety takes priority over convenience.

Recommended order:

1. start from a clean Python session
2. create only the parts needed for the current test
3. center steering before and after motion tests
4. stop throttle explicitly before shutdown
5. call `shutdown()` on parts before recreating them
6. test that shutdown really stopped the car
7. avoid multiple overlapping Pico users in different sessions

## Shutdown rules

Before ending a session:

1. send stop throttle
2. center steering
3. call part shutdown methods
4. verify that wheels are no longer moving
5. then exit Python / kill tmux

Do not assume interpreter exit alone leaves the car safe.
Do not assume killing tmux or Python is equivalent to a safe stop.
For throttle-related scripts, the script must be written so that exit
paths always run a stop plus `shutdown()` sequence.

## Known pitfalls

- Recreating parts without `shutdown()` can cause Pico serial conflicts.
- Background print loops can pollute the REPL and make commands hard to
  enter safely.
- Killing a session without an explicit throttle stop may leave the car
  moving.
- Recreating parts without a prior shutdown can leave actuators in an
  unexpected state.
- Using multiple concurrent Pico controllers can trigger serial errors.
- The Pico data port may not always come back as `/dev/ttyACM1`; code
  now needs to tolerate fallback to another `/dev/ttyACM*` device.
- Long runs can coincide with USB disconnect/reconnect and undervoltage
  events, so hardware/power stability must be checked alongside software.

## Helper script

A safer manual test helper now exists at:

- `scripts/ai/manual_hw_test.py`
- `scripts/ai/manual_motion_control.py`

Important current state of `manual_motion_control.py`:

- safe cleanup exists
- pulse-based motion control exists
- adaptive pulse escalation has been started
- short bench distance runs work
- low-speed long-distance runs are still a pending problem

Examples:

```bash
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py stop
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py center
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py steering --angle -1
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py throttle --value 0.15
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py led --mode 0 --seconds 3
```

## Practical reminders

Use the env Python explicitly:

```bash
/home/dirk/env/bin/python
```

Center steering:

```python
steering.run(0)
```

Stop throttle:

```python
throttle.run(0)
```

If needed, send the configured stop pulse through the throttle
controller path before shutdown.
