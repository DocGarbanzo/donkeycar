# manual motion control plan for donkey5

This document describes the intended safe approach for using the
`donkey5` throttle driver together with `OdometerPico` so that an AI
helper script can do tasks such as:

- drive at `0.3 m/s`
- drive forward for `3 m`
- stop safely on exit or interruption

## Goals

1. safety first
2. reliable stop on every exit path
3. predictable speed control
4. predictable distance-stop behavior
5. no dirty REPL state or overlapping Pico sessions

## Non-goals

This is not a full autonomous driving stack.

Initial scope is:

- centered steering
- forward motion only
- one active motion script at a time
- manual invocation from `scripts/ai/`

## Safety requirements

These are mandatory.

### 1. Always stop throttle before exit

Every motion script must guarantee:

- `throttle.run(0)` is sent
- `throttle.shutdown()` is called
- this happens on:
  - normal completion
  - exceptions
  - `KeyboardInterrupt`
  - `SIGTERM`
  - explicit abort command

### 2. Always center steering before and after motion tests

The script should:

- center steering on startup
- keep steering centered unless a steering feature is explicitly added
- center steering again before shutdown

### 3. Verify shutdown behavior in testing

It is not enough to call `shutdown()` in code.

During testing we must verify physically that:

- wheels stop turning
- steering returns to center
- Pico comms are not left in a dirty state

### 4. One Pico owner at a time

Only one script/session should own the Pico-backed parts.

Do not:

- reuse polluted interactive sessions
- recreate throttle/steering/odometer parts without shutdown
- run multiple motion helpers concurrently

## Parts involved

### Steering

- `PWMSteering`
- `PulseController`
- `PwmPinPico`
- `STEERING_CHANNEL = "PICO.BCM.16"`

Steering will stay centered for initial motion-control work.

### Throttle

- `PWMThrottle`
- `PulseController`
- `PwmPinPico`
- `THROTTLE_CHANNEL = "PICO.BCM.17"`
- `THROTTLE_STOPPED_PWM = 370`

### Odometer

- `OdometerPico`
- `ODOMETER_GPIO = "PICO.BCM.2"`
- `TICK_PER_M = 75`
- `ODOMETER_USE_PIO = True`
- `ODOMETER_FREQUENCY = 20000`

## Implementation strategy

Implementation should happen in stages.

### Stage 1: clean harness

Create a dedicated script, likely:

- `scripts/ai/manual_motion_control.py`

The script should:

- use `/home/dirk/env/bin/python`
- create steering, throttle, and odometer exactly once
- register cleanup with `atexit`
- install signal handlers for `SIGINT` and `SIGTERM`
- stop throttle and center steering in cleanup

### Stage 2: odometer validation

Before closing the loop, validate the sensor.

Add a mode like:

```bash
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py observe
```

This mode should:

- keep throttle at zero
- print `speed`, `inst_speed`, and `distance`
- optionally allow a fixed throttle step test
- check whether readings are bursty, delayed, or plausible

We need to confirm:

- distance increases monotonically
- speed estimates are plausible
- no obvious scaling bug
- zero-speed timeout works as intended

### Stage 3: fixed-throttle characterization

Updated bench testing with wheels in the air:

- `0.20` produced no odometer movement
- `0.25` produced clear forward wheel motion
- `0.30` produced faster forward wheel motion
- the earlier `0.13` / `0.15` assumption was wrong for the current rig

This confirms that the key parameter is the actual measured motion
threshold on this hardware, not the earlier estimate.

Because unloaded wheel speed is much higher than on-ground vehicle speed,
control should be built around threshold-crossing pulses rather than a
continuous low throttle estimate.

Add a mode like:

```bash
... manual_motion_control.py throttle --value 0.20
... manual_motion_control.py throttle --value 0.25
... manual_motion_control.py throttle --value 0.30
... manual_motion_control.py threshold --start 0.20 --stop 0.35 \
    --step 0.05 --seconds 5
```

Measure:

- steady-state speed
- startup latency
- overshoot
- whether odometer values come in bursts

Output should be logged clearly so we can build a rough map:

- throttle -> typical speed
- throttle -> sustained-motion candidate

That map can provide a feed-forward estimate.
The `threshold` command is intended to automate the missing check for the
lowest candidate that appears to move continuously instead of stuttering.

### Stage 4: speed-hold controller

Add a mode like:

```bash
... manual_motion_control.py speed --target 0.3
```

Current approach uses pulse control instead of continuous low throttle.

Controller design:

- steering fixed at center
- 20 Hz control loop (`0.05 s`)
- maintain a progress debt in meters
- when debt exceeds a threshold, send a short throttle pulse above the
  motion threshold
- then coast and observe odometer feedback

Initial pulse parameters from updated bench work:

- pulse throttle: about `0.25`
- pulse on-time: about `0.20 s`
- minimum gap between pulses: about `0.10 s`

This works better than trying to hold a continuous throttle below the
motion threshold.

Safeguards:

- pulse throttle must still be clamped to a safe range
- if measured speed is invalid or implausible, drop to zero
- if no odometer updates for too long, drop to zero

### Stage 5: distance-stop controller

**Verified approach (bench confirmed):**

```bash
... manual_motion_control.py distance --meters 10
```

Approach:

1. record starting odometer distance
2. apply continuous minimum throttle (`--min-thr`, default `0.23`)
3. read odometer in a tight 20 Hz loop
4. stop throttle the moment `traveled >= target`
5. accept a small overshoot from residual momentum

Key design decisions learned from failed attempts:

- **Do not pulse the throttle** on and off during distance runs —
  the on/off pattern is visibly stuttery
- **Never drop below the minimum throttle** while motion is intended —
  any dip to zero causes a stutter
- **Do not send steering commands** — centering steering on startup
  produces a visible jerk; leave steering alone for distance runs
- The small overshoot (typically 1–8 cm on bench) is acceptable

The `--speed` parameter and pulse-based braking zone have been removed
from the distance command. The `--min-thr` parameter controls the
single throttle value sent for the entire run.

## Handling bursty odometer readings

This is the main reason the first ad-hoc attempt was poor.

Potential issues observed:

- readings alternate between zero and large values
- control reacts too aggressively to spikes
- controller toggles between throttle and zero

Mitigations for speed mode (pulse controller):

1. do not use raw instantaneous speed alone
2. prefer averaged speed from `OdometerPico`
3. reject obviously implausible jumps
4. keep proportional gain small initially
5. rely on feed-forward plus mild correction
6. optionally low-pass filter the measured speed again in the script

For distance mode the bursty readings are not a control problem —
the odometer distance counter is cumulative and accurate even when
the per-cycle speed estimate is noisy.  The script simply stops when
the cumulative distance reaches the target.

If needed, a second script-level smoothed speed signal can be added:

```text
filtered_speed = alpha * measured_speed + (1 - alpha) * filtered_speed
```

## Logging and observability

The motion script should print structured status lines such as:

```text
mode=speed target=0.30 speed=0.28 inst=0.31 dist=1.42 thr=0.14
```

For distance mode:

```text
mode=distance target_m=3.00 remain=0.42 speed=0.19 dist=2.58 thr=0.08
```

This makes tuning and debugging easier.

## Abort behavior

The script should support immediate abort.

Possible forms:

- `Ctrl+C`
- signal termination
- a future `stop` subcommand

Abort must do:

1. throttle zero
2. steering center
3. part shutdown
4. exit

## Proposed script commands

```bash
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py observe
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py throttle --value 0.23
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py speed --target 0.3
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py distance --meters 3
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py distance --meters 10 --min-thr 0.23
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py stop
```

## Acceptance criteria

### Speed mode

Acceptable first result:

- can hold near `0.3 m/s`
- does not oscillate violently
- exits safely

### Distance mode

Acceptable first result:

- can drive forward about `3 m`
- stops within a reasonable tolerance
- exits safely

### Safety

Mandatory result:

- no script exit leaves the motor spinning
- distance command does not touch steering; other commands center on
  startup and cleanup

## Implementation status

Implemented script:

- `scripts/ai/manual_motion_control.py`

Implemented commands:

- `stop`
- `center`
- `steering`
- `observe`
- `throttle`
- `threshold`
- `speed`
- `distance`

Implemented safety behavior:

- `atexit` cleanup
- signal cleanup for `SIGINT` and `SIGTERM`
- explicit throttle stop on shutdown
- steering center on startup and shutdown (except `distance`, which
  sends no steering commands at all)
- bounded automatic Pico recovery via `--recoveries`
- throttle clamped to `--throttle-limit`
- stop on repeated odometer read errors

Distance command design (verified approach):

- continuous minimum throttle throughout the run
- 20 Hz odometer poll loop; stop the moment cumulative distance reaches
  the target
- no steering commands sent — avoids startup jerk
- no pulsing — avoids stuttery motion
- accepts small overshoot from residual momentum (typically < 10 cm
  on bench)
- `--min-thr` sets the single throttle value (default `PULSE_THR = 0.23`;
  on-ground minimum confirmed at `0.17`)

Speed command design (pulse controller, bench-validated only):

- debt accumulator tracks distance owed vs. distance traveled
- odometer-gated pulse: throttle on until odometer reports debt repaid
  or `--pulse-on-s` maximum elapses
- adaptive escalation: raises pulse throttle only when previous pulse
  produced no motion (not on every cycle)
- `--pulse-thr`, `--pulse-thr-max`, `--pulse-on-s` are tunable at
  runtime

Pico robustness:

- `donkeycar.parts.pico.Pico` falls back to any available `/dev/ttyACM*`
  device (also overridable via `DONKEY_PICO_PORT` env var)
- `get_pico()` / `reset_pico()` manage the global singleton
- `_wait_until_ready()` with timeout replaces the old infinite spin

## Bench results so far

### Odometer observation

Observed at zero commanded throttle:

- occasional non-zero odometer spikes can happen immediately after prior
  movement
- after settling, speed returns to zero as expected

### Fixed throttle characterization

Bench findings with wheels in the air after switching to direct
non-threaded throttle commands:

- `0.20`: no sustained motion
- `0.23`: minimum forward-motion threshold (confirmed)
- `0.25`: clear sustained forward motion
- `0.30`: faster sustained forward motion
- the key useful result is the threshold, not the unloaded speed

### Speed mode

A `speed --target 0.3` bench test worked with pulse control.
The controller pulsed throttle and reacted to odometer feedback.
This demonstrated the mechanism, but it is not yet claimed to be tuned
for on-ground `0.3 m/s`.

### Distance mode — pulse approach (abandoned)

Earlier pulse-based distance runs were visibly bad:

- on/off throttle pattern produced stuttery motion
- centering steering on startup caused a visible jerk
- pulse throttle escalated to maximum, spinning wheels too fast

### Distance mode — continuous throttle approach (verified)

Command:

```bash
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py distance --meters 10
```

Results (bench, wheels in air):

| Run | Target | Stopped at | Overshoot |
|-----|--------|------------|-----------|
| 1   | 10.00 m | 10.013 m | 1.3 cm |
| 2   | 10.00 m | 9.973 m  | — |
| 3   | 10.00 m | 10.080 m | 8.0 cm |

Motion was smooth and continuous. No steering jerk. Visually confirmed
as good by user.

## Known blockers

1. Pico comms can produce occasional JSON parse errors on long runs
   (non-fatal; the odometer read simply returns stale data that cycle).
2. Kernel undervoltage events can occur during intensive runs.
3. Bench behavior with wheels in the air is useful for threshold
   discovery, but not enough to finalize on-ground speed tuning.

## Recommended next restart point

After context reset, resume from this order:

1. read this file and `docs/donkey5-hardware-control.md`
2. verify Pico device availability (`/dev/ttyACM*`)
3. run `stop` and optionally `center` first
4. run a short `observe` to confirm odometer is live
5. run `distance --meters 2` on the ground as a smoke test
6. tune `--min-thr` if needed for on-ground friction

## Current completion state

What is confirmed:

- steering over Pico PWM works
- direct non-threaded throttle commands over Pico PWM work
- odometer feedback is accurate for cumulative distance
- minimum forward-motion throttle threshold (bench, wheels in air): **0.23**
- minimum forward-motion throttle threshold (on-ground): **0.17**
- continuous-throttle distance command is smooth and visually confirmed
- 10 m bench runs complete within ~8 cm of target
- all safety requirements met: stop on exit, no steering jerk,
  clean Pico shutdown

Remaining work:

1. ~~implement all script commands~~ — DONE
2. ~~fix stuttery pulse-based distance control~~ — DONE (replaced with
   continuous throttle)
3. ~~fix steering jerk on startup~~ — DONE (distance sends no steering)
4. ~~on-ground calibration of `--min-thr` for actual vehicle weight~~ —
   DONE: on-ground minimum is **0.17**; `0.16` is marginal, `0.15` fails
5. tune on-ground overshoot (may need slightly earlier stop threshold)
6. verify behavior under real power conditions (not bench PSU)
