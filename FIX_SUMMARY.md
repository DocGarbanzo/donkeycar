# Fix for Failing Test Issue #8

## Problem
The test `pico/circuit/test_pio_pulse.py::test_configuration` was failing in CI with the error:
```
serial.serialutil.SerialException: [Errno 2] could not open port /dev/ttyACM1: 
[Errno 2] No such file or directory: '/dev/ttyACM1'
```

## Root Cause Analysis

### Which commit introduced the error?
The issue was introduced in commit **b6bfd05457d9b2eed57f1bdf0356aeccb1c9cbdd** (Aug 26, 2024) on the `new_dev` branch with the message:
> "First commit to integrate Pico as another pin provider instead of being a donkeycar part."

This commit added a module-level instance creation in `donkeycar/parts/pico.py` at line 300:
```python
instance = Pico()
```

### Why did it fail?
The `Pico()` constructor immediately tries to open a serial connection to `/dev/ttyACM1` in its `__init__` method:
```python
def __init__(self, port: str = "/dev/ttyACM1"):
    self.serial = serial.Serial(port, 115200)
```

This module-level instantiation means that simply importing the module (which happens when the test tries to `from donkeycar.parts.pico import instance`) triggers the serial port connection attempt. In CI environments where the Pico hardware is not available, this causes an immediate failure before any test code can run.

## Solution
The fix involves two changes:

### 1. Make Pico instance creation conditional (`donkeycar/parts/pico.py`)
Wrap the instance creation in a try-except block to handle environments where the hardware is not available:

```python
# Try to create the Pico instance, but if the hardware is not available
# (e.g., in test/CI environments), set instance to None
try:
    instance = Pico()
except (serial.SerialException, FileNotFoundError, RuntimeError) as e:
    logger.warning(
        f"Could not create Pico instance: {e}. "
        "Pico hardware may not be available. Tests can still run."
    )
    instance = None
```

### 2. Skip test when hardware is unavailable (`pico/circuit/test_pio_pulse.py`)
Update the test to gracefully skip when the hardware is not present:

```python
def test_configuration():
    import pytest
    from donkeycar.parts.pico import instance as pico
    
    # Skip test if hardware is not available
    if pico is None:
        pytest.skip("Pico hardware not available")
    
    # ... rest of test code
```

## Implementation
The fix has been committed to the `new_dev` branch in commit **b2bf00db**.

## Impact
- Tests can now run in CI environments without Pico hardware
- The test properly skips when hardware is unavailable rather than failing
- Pico functionality remains unchanged when hardware IS available
- No breaking changes to the API

## Branch Status
- The failing test only exists on the `new_dev` branch
- The fix is applied to the `new_dev` branch
- When `new_dev` is merged to `main`, the fix will be included
- The `main` branch does not currently have the Pico code, so there are no failing tests there

## Verification
The fix can be verified by running:
```bash
pytest pico/circuit/test_pio_pulse.py::test_configuration -v
```

In environments without hardware, the test should now skip with message "Pico hardware not available" instead of failing with a serial exception.
