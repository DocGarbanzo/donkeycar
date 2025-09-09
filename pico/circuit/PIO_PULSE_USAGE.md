# PIO-based Pulse Input Documentation

## Overview

The new PIO-based pulse input (`PULSE_IN_PIO`) provides high-precision, hardware-accelerated pulse width measurement using the Raspberry Pi Pico's Programmable I/O (PIO) system.

## Key Improvements over Regular PULSE_IN

1. **Hardware-based timing**: Uses PIO state machine for precise timing instead of software polling
2. **Configurable frequency**: Allows custom clock frequencies for different timing requirements
3. **Non-blocking**: Continuous measurement without blocking main CPU
4. **Higher precision**: Microsecond resolution with automatic frequency compensation
5. **Larger buffer**: Default 64-element buffer vs 2-element for regular pulse input

## Usage Examples

### Basic Usage (Python)

```python
from donkeycar.parts.pins import pulse_in_pin_by_id

# Create PIO-based pulse input pin
pulse_pin = pulse_in_pin_by_id("PICO.BCM.15", maxlen=64, 
                               auto_clear=True, use_pio=True)

# Start pulse measurement
pulse_pin.start(maxlen=64, auto_clear=True)

# Read pulse widths
while True:
    pulses = pulse_pin.read_pulses()
    if pulses:
        print(f"Pulse widths: {pulses} microseconds")
    time.sleep(0.1)

# Stop when done
pulse_pin.stop()
```

### Configuration via Pico Setup

```python
from donkeycar.parts.pico import instance as pico

# Setup PIO pulse input with custom frequency
pico.setup_input_pin('GP15', 'PULSE_IN_PIO', 
                     maxlen=64, 
                     auto_clear=True,
                     frequency=2_000_000)  # 2MHz for 0.5μs resolution

# Read measurements
pulse_data = pico.read('GP15')
```

### JSON Configuration for Pico

```json
{
  "input_pins": {
    "GP15": {
      "mode": "PULSE_IN_PIO",
      "maxlen": 64,
      "auto_clear": true,
      "frequency": 2000000
    }
  }
}
```

## Frequency and Timing Accuracy

### Timing Resolution
- **Default 2MHz**: ~0.5μs resolution (each count = 0.5μs due to 2-cycle PIO loop)
- **1MHz**: ~1.0μs resolution  
- **Custom**: Resolution = 1,000,000 / (frequency/2) microseconds

### Frequency Considerations

1. **Requested vs Actual**: The PIO may not achieve the exact requested frequency
2. **Auto-scaling**: Implementation automatically compensates for actual frequency
3. **Accuracy factors**:
   - Crystal oscillator accuracy (typically ±50ppm)
   - Temperature stability
   - System clock dividers

### Example Frequency Configurations

```python
# High resolution (0.25μs) - may not be achievable on all pins
high_res = {'mode': 'PULSE_IN_PIO', 'frequency': 4_000_000, 'maxlen': 32}

# Standard resolution (0.5μs) - recommended default
standard = {'mode': 'PULSE_IN_PIO', 'frequency': 2_000_000, 'maxlen': 64}

# Lower resolution (2μs) - for slower signals, better stability
low_res = {'mode': 'PULSE_IN_PIO', 'frequency': 500_000, 'maxlen': 128}
```

## Performance Comparison

| Feature | Regular PULSE_IN | PIO PULSE_IN_PIO |
|---------|------------------|-------------------|
| Resolution | Variable (software dependent) | Configurable (0.5μs typical) |
| CPU Usage | High (polling) | Low (hardware) |
| Buffer Size | 2-8 pulses | 64+ pulses |
| Accuracy | ±several μs | ±0.5μs |
| Blocking | Yes | No |
| Frequency Range | Limited by software | Limited by PIO clock |

## Use Cases

### High-Speed Odometry
```python
# For wheel encoders with high pulse rates
odometer_pin = pulse_in_pin_by_id("PICO.BCM.16", maxlen=128, 
                                 auto_clear=True, use_pio=True)
```

### RC Servo Signal Reading
```python  
# For precise servo signal measurement (1000-2000μs pulses)
servo_pin = pulse_in_pin_by_id("PICO.BCM.17", maxlen=8,
                              auto_clear=True, use_pio=True)
```

### Ultrasonic Distance Sensors
```python
# For echo pulse measurement (HC-SR04, etc)
echo_pin = pulse_in_pin_by_id("PICO.BCM.18", maxlen=4,
                             auto_clear=False, use_pio=True)
```

## Troubleshooting

### Common Issues

1. **Frequency Warning**: "Warning: Requested frequency 2000000Hz, actual 1999847Hz"
   - This is normal - implementation auto-compensates
   - Difference >1% may indicate clock issues

2. **No Pulse Data**: Empty pulse arrays returned
   - Check pin connections
   - Verify signal voltage levels (3.3V logic)
   - Confirm pin number matches GPIO number

3. **Timing Inaccuracy**: Measured pulse widths don't match expected
   - Verify frequency setting matches requirements
   - Check for electromagnetic interference
   - Consider lower frequency for better stability

### Testing Timing Accuracy

```python
# Generate test signal and measure accuracy
import time
from donkeycar.parts.pins import pwm_pin_by_id, pulse_in_pin_by_id

# Output 1500μs PWM pulse on GP14
pwm_out = pwm_pin_by_id("PICO.BCM.14")
pwm_out.start(0.075)  # 1500μs at 60Hz

# Measure on GP15  
pulse_in = pulse_in_pin_by_id("PICO.BCM.15", use_pio=True)
pulse_in.start()

# Compare measured vs expected
for i in range(10):
    pulses = pulse_in.read_pulses()
    if pulses:
        measured = [p for p in pulses if 1000 < p < 2000]  # Filter high pulses
        if measured:
            avg_width = sum(measured) / len(measured)
            error = abs(avg_width - 1500)
            print(f"Expected: 1500μs, Measured: {avg_width:.1f}μs, Error: {error:.1f}μs")
    time.sleep(0.5)
```

## Migration from Regular PULSE_IN

To upgrade existing code from regular `PULSE_IN` to `PULSE_IN_PIO`:

1. **Change pin mode**:
   ```python
   # Old
   pin_config = {'mode': 'PULSE_IN', 'maxlen': 4}
   
   # New  
   pin_config = {'mode': 'PULSE_IN_PIO', 'maxlen': 64, 'frequency': 2_000_000}
   ```

2. **Update buffer sizes**: Consider larger buffers (32-128) for PIO version

3. **Add frequency parameter**: Choose appropriate frequency for your timing requirements

4. **Test timing**: Verify accuracy meets your application needs

The PIO implementation is backward-compatible with the existing `PulseInPin` interface, so no code changes are required beyond configuration.