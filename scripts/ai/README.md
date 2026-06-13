# AI helper scripts

This folder contains helper scripts created during AI-assisted work.

These scripts are intended to make manual testing safer and more
repeatable than ad-hoc interactive REPL sessions.

## Current scripts

Design and planning docs live under `docs/`.
Relevant documents include:

- `docs/donkey5-hardware-control.md`
- `docs/manual-motion-control-plan.md`

### `manual_hw_test.py`

Manual hardware helper for the `donkey5` setup.

Examples:

```bash
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py stop
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py center
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py steering --angle -1
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py throttle --value 0.15
/home/dirk/env/bin/python scripts/ai/manual_hw_test.py led --mode 0 --seconds 3
```

### `manual_motion_control.py`

Safety-first motion helper for observing odometer values, sending fixed
throttle, trying speed hold, and trying distance-stop.

Current status:

- safe cleanup implemented
- threshold-based pulse controller implemented
- short bench tests work
- long low-speed runs are not fully solved yet
- on-ground minimum throttle confirmed: **0.17** (0.16 marginal, 0.15 fails)

Examples:

```bash
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py stop
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py observe --seconds 5
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py throttle --value 0.15 --seconds 2
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py speed --target 0.3 --seconds 5
/home/dirk/env/bin/python scripts/ai/manual_motion_control.py distance --meters 3 --speed 0.3
```

## Safety notes

- Stop throttle before ending a session.
- Center steering after manual tests.
- Prefer these scripts over reusing dirty interactive sessions.
- Avoid recreating Pico-backed parts without calling `shutdown()`.
