#!/usr/bin/env python3
"""
Test script to compare regular PULSE_IN vs PIO-based PULSE_IN_PIO
"""

import time
from donkeycar.parts.pins import pulse_in_pin_by_id

def test_pulse_implementations():
    """
    Test both pulse input implementations
    """
    print("Testing Pulse Input Implementations")
    print("=" * 40)
    
    # Test parameters
    pin_id = "PICO.BCM.15"  # GPIO 15
    maxlen = 8
    auto_clear = True
    test_duration = 5.0  # seconds
    
    print(f"Pin ID: {pin_id}")
    print(f"Max length: {maxlen}")
    print(f"Auto clear: {auto_clear}")
    print(f"Test duration: {test_duration}s")
    print()
    
    # Test regular pulse input
    print("1. Testing regular PULSE_IN implementation:")
    try:
        regular_pin = pulse_in_pin_by_id(pin_id, maxlen=maxlen, 
                                       auto_clear=auto_clear, use_pio=False)
        regular_pin.start(maxlen=maxlen, auto_clear=auto_clear)
        
        start_time = time.time()
        pulse_count = 0
        
        while time.time() - start_time < test_duration:
            pulses = regular_pin.read_pulses()
            if pulses:
                pulse_count += len(pulses)
                print(f"  Regular: {len(pulses)} pulses, widths: {pulses}")
            time.sleep(0.1)
        
        regular_pin.stop()
        print(f"  Total pulses detected (regular): {pulse_count}")
        
    except Exception as e:
        print(f"  Regular implementation error: {e}")
    
    print()
    
    # Test PIO pulse input
    print("2. Testing PIO PULSE_IN_PIO implementation:")
    try:
        pio_pin = pulse_in_pin_by_id(pin_id, maxlen=maxlen, 
                                   auto_clear=auto_clear, use_pio=True)
        pio_pin.start(maxlen=maxlen, auto_clear=auto_clear)
        
        start_time = time.time()
        pulse_count = 0
        
        while time.time() - start_time < test_duration:
            pulses = pio_pin.read_pulses()
            if pulses:
                pulse_count += len(pulses)
                print(f"  PIO: {len(pulses)} pulses, widths: {pulses}")
            time.sleep(0.1)
        
        pio_pin.stop()
        print(f"  Total pulses detected (PIO): {pulse_count}")
        
    except Exception as e:
        print(f"  PIO implementation error: {e}")

def test_configuration():
    """
    Test pin configuration through the Pico interface
    """
    print("\n" + "=" * 40)
    print("Testing Pin Configuration")
    print("=" * 40)
    
    from donkeycar.parts.pico import instance as pico
    
    # Test regular PULSE_IN setup
    print("Testing regular PULSE_IN setup...")
    try:
        pico.setup_input_pin('GP16', 'PULSE_IN', maxlen=4, auto_clear=False)
        print("  ✓ Regular PULSE_IN setup successful")
        pico.remove_pin('GP16')
    except Exception as e:
        print(f"  ✗ Regular PULSE_IN setup failed: {e}")
    
    # Test PIO PULSE_IN setup with default frequency
    print("Testing PIO PULSE_IN_PIO setup (default 2MHz)...")
    try:
        pico.setup_input_pin('GP17', 'PULSE_IN_PIO', maxlen=64, auto_clear=False)
        print("  ✓ PIO PULSE_IN_PIO setup successful")
        pico.remove_pin('GP17')
    except Exception as e:
        print(f"  ✗ PIO PULSE_IN_PIO setup failed: {e}")
    
    # Test PIO PULSE_IN setup with custom frequency  
    print("Testing PIO PULSE_IN_PIO setup (custom 1MHz)...")
    try:
        pico.setup_input_pin('GP18', 'PULSE_IN_PIO', maxlen=64, 
                           auto_clear=False, frequency=1_000_000)
        print("  ✓ PIO PULSE_IN_PIO custom frequency setup successful")
        pico.remove_pin('GP18')
    except Exception as e:
        print(f"  ✗ PIO PULSE_IN_PIO custom frequency setup failed: {e}")

def test_frequency_accuracy():
    """
    Test frequency accuracy and scaling
    """
    print("\n" + "=" * 40)
    print("Testing Frequency Accuracy")
    print("=" * 40)
    
    # This would require actual hardware testing
    print("Frequency accuracy testing requires:")
    print("1. Hardware Pico connected with CircuitPython")
    print("2. Known frequency PWM signal source")
    print("3. Measurement of actual vs expected pulse widths")
    print()
    print("Key points about timing accuracy:")
    print("- PIO clock may not achieve exact requested frequency")
    print("- Clock source accuracy depends on Pico crystal oscillator")
    print("- Temperature and voltage affect clock stability")
    print("- The implementation now auto-scales based on actual frequency")
    print("- Frequency verification warnings help detect timing issues")

if __name__ == "__main__":
    try:
        test_configuration()
        test_frequency_accuracy()
        print("\nNote: Pulse width testing requires actual pulse signals.")
        print("Connect a PWM source to GPIO 15 to see pulse measurements.")
        test_pulse_implementations()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
    finally:
        print("Test completed")