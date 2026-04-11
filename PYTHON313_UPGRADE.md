# Python 3.13 Upgrade Plan

Upgrade donkeycar from Python 3.11 to support Python 3.13, required for
Raspberry Pi 5 running Debian Trixie.

## Status Legend
- [ ] Pending
- [x] Complete

---

## Step 1: Update `setup.cfg` — Package Metadata and Dependencies
- [x] `python_requires`: change `>=3.11.0,<3.12` → `>=3.13.0,<3.14` (single version)
- [x] Update classifier to Python 3.13 only
- [x] `numpy`: bump lower bound to `>=1.26.0` (required by TF 2.20)
- [x] `pc` extra: `tensorflow==2.15.*` → `tensorflow==2.20.*`, add `tf-keras==2.20.*`
- [x] `macos` extra: same TF changes as pc
- [x] `pi` extra: remove `tflite-runtime`, add `ai-edge-litert>=2.1.4`
- [x] `pi` extra: remove `flatbuffers==24.3.*` (ai-edge-litert pulls its own)
- [x] `torch` extra: `torch==2.1.*` → `torch==2.6.*`, `torchvision==0.21.*`, `torchaudio==2.6.*`

## Step 2: Fix `tensorflow.python.*` Internal API Imports

These internal paths were removed in TF 2.16+.

- [x] `donkeycar/parts/interpreter.py`: restore `get_tflite_interpreter()` with
  `ai_edge_litert` fallback; fix `tensorflow.python.saved_model` imports; wrap
  TF imports in try/except
- [x] `donkeycar/parts/keras.py`: wrap TF imports in try/except; replace
  `DatasetV1/DatasetV2` type hints with `Any`
- [x] `donkeycar/pipeline/training.py`: fix
  `from tensorflow.python.keras.models import load_model`
- [x] `donkeycar/parts/keras_2.py`: fix
  `from tensorflow.python.keras.layers import Activation`
- [x] `donkeycar/management/makemovie.py`: fix 3x `tensorflow.python.keras` imports
- [x] `donkeycar/management/base.py`: fix `tensorflow.python.keras.models` import

## Step 3: Update GitHub Actions CI
- [x] Add Python 3.13 to test matrix in
  `.github/workflows/python-package-conda.yml`

## Step 4: Update CLAUDE.md
- [x] Update Python version requirement from `3.11+ but < 3.12` to `3.11–3.13`

## Step 5: Testing
- [x] Run full test suite on Ubuntu with existing Python 3.11 (regression check)
- [x] SSH to Pi, create venv with Python 3.13 (--system-site-packages for libcamera), install `.[pi]`
- [x] Verify TFLite via `ai-edge-litert` on Pi
- [x] Verify camera and GPIO imports on Pi

---

## Key Decisions

**TensorFlow:** TF 2.20 is the first release with Python 3.13 wheels.
TF 2.16+ uses standalone Keras 3; the `tf-keras` shim restores the Keras 2
API as `tensorflow.keras.*`. All `tensorflow.python.keras.*` imports must be
replaced with `tensorflow.keras.*`.

**TFLite on Pi:** `tflite-runtime` is dead (last release 2.14, Python 3.11
max). Replaced by `ai-edge-litert` (Google's official successor, drop-in API).

**PyTorch:** `torch==2.6.*` is the first series with Python 3.13 aarch64
wheels. `torchvision==0.21.*` and `torchaudio==2.6.*` match.

**RPi.GPIO vs gpiozero:** Do NOT follow autorope/main switching to `RPi.GPIO`
(wheels stop at Python 3.9). Keep `gpiozero` which supports Python 3.13.

**Nano extra:** Leave numpy/matplotlib/pandas pins — they reflect Jetson Nano
hardware constraints, not Python version limitations.
