#!/usr/bin/env python3
"""
Hardware test for PIO pulse input implementation
Requires physical jumper wire between GP0 (PWM output) and GP2 (pulse input)
"""

import time
from donkeycar.parts.pins import pwm_pin_by_id, pulse_in_pin_by_id

def test_pulse_accuracy():
    """
    Test pulse accuracy by generating known PWM signals and measuring them
    """
    print("PIO Pulse Input Hardware Test")
    print("=" * 50)
    print("SETUP: Connect jumper wire between GP0 and GP2")
    print("GP0 = PWM output (pulse generator)")
    print("GP2 = Pulse input (measurement)")
    print("=" * 50)
    
    # Test parameters
    pwm_pin = "PICO.BCM.0"     # GPIO 0 for PWM output
    pulse_pin = "PICO.BCM.2"   # GPIO 2 for pulse input
    
    # Test different pulse widths (in microseconds)
    test_pulses = [
        (1000, 0.06),    # 1000μs pulse at 60Hz = 6% duty cycle
        (1500, 0.09),    # 1500μs pulse at 60Hz = 9% duty cycle  
        (2000, 0.12),    # 2000μs pulse at 60Hz = 12% duty cycle
        (800, 0.048),    # 800μs pulse at 60Hz = 4.8% duty cycle
        (2200, 0.132),   # 2200μs pulse at 60Hz = 13.2% duty cycle
    ]
    
    pwm_out = None
    pulse_regular = None
    pulse_pio = None
    
    try:
        # Setup PWM output on GP0
        print("Setting up PWM output on GP0...")
        pwm_out = pwm_pin_by_id(pwm_pin, frequency_hz=60)
        
        print("Testing both pulse input implementations...")
        print()
        
        for expected_us, duty_cycle in test_pulses:
            print(f"Testing {expected_us}μs pulse (duty cycle: {duty_cycle:.3f})")
            print("-" * 40)
            
            # Start PWM with specified duty cycle
            pwm_out.start(duty_cycle)
            time.sleep(0.5)  # Let PWM stabilize
            
            # Test 1: Regular pulse input
            print("1. Regular PULSE_IN measurement:")
            try:
                pulse_regular = pulse_in_pin_by_id(pulse_pin, maxlen=16, 
                                                 auto_clear=True, use_pio=False)
                pulse_regular.start(maxlen=16, auto_clear=True)
                
                # Collect measurements
                measurements = []
                for i in range(10):  # 10 samples
                    pulses = pulse_regular.read_pulses()
                    if pulses:
                        # Filter for high pulses (PWM high time)
                        high_pulses = [p for p in pulses if p > 500]
                        measurements.extend(high_pulses)
                    time.sleep(0.1)
                
                pulse_regular.stop()
                
                if measurements:
                    avg_regular = sum(measurements) / len(measurements)
                    error_regular = abs(avg_regular - expected_us)
                    error_pct_regular = (error_regular / expected_us) * 100
                    print(f"   Samples: {len(measurements)}")
                    print(f"   Average: {avg_regular:.1f}μs")
                    print(f"   Error: {error_regular:.1f}μs ({error_pct_regular:.1f}%)")
                else:
                    print("   No measurements captured")
                    
            except Exception as e:
                print(f"   Regular pulse input error: {e}")
            
            time.sleep(0.2)
            
            # Test 2: PIO pulse input
            print("2. PIO PULSE_IN_PIO measurement:")
            try:
                pulse_pio = pulse_in_pin_by_id(pulse_pin, maxlen=64, 
                                             auto_clear=True, use_pio=True)
                pulse_pio.start(maxlen=64, auto_clear=True)
                
                # Collect measurements
                measurements = []
                for i in range(10):  # 10 samples
                    pulses = pulse_pio.read_pulses()
                    if pulses:
                        # Filter for high pulses (PWM high time)
                        high_pulses = [p for p in pulses if p > 500]
                        measurements.extend(high_pulses)
                    time.sleep(0.1)
                
                pulse_pio.stop()
                
                if measurements:
                    avg_pio = sum(measurements) / len(measurements)
                    error_pio = abs(avg_pio - expected_us)
                    error_pct_pio = (error_pio / expected_us) * 100
                    print(f"   Samples: {len(measurements)}")
                    print(f"   Average: {avg_pio:.1f}μs")
                    print(f"   Error: {error_pio:.1f}μs ({error_pct_pio:.1f}%)")
                else:
                    print("   No measurements captured")
                    
            except Exception as e:
                print(f"   PIO pulse input error: {e}")
            
            print()
            
        # Stop PWM
        pwm_out.stop()
        
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        # Cleanup
        if pwm_out:
            try:
                pwm_out.stop()
            except:
                pass
        if pulse_regular:
            try:
                pulse_regular.stop()
            except:
                pass
        if pulse_pio:
            try:
                pulse_pio.stop()
            except:
                pass

def test_frequency_response():
    """
    Test both implementations at different PWM frequencies
    """
    print("\n" + "=" * 50)
    print("Testing Frequency Response")
    print("=" * 50)
    print("Fixed 1500μs pulse width at different frequencies")
    
    pwm_pin = "PICO.BCM.0"
    pulse_pin = "PICO.BCM.2"
    
    # Test different frequencies
    test_frequencies = [30, 60, 100, 200]  # Hz
    target_pulse_us = 1500
    
    pwm_out = None
    
    try:
        for freq_hz in test_frequencies:
            print(f"\nTesting at {freq_hz}Hz:")
            
            # Calculate duty cycle for 1500μs pulse
            period_us = 1_000_000 / freq_hz
            duty_cycle = target_pulse_us / period_us
            
            if duty_cycle > 0.95:  # Skip if duty cycle too high
                print(f"   Skipped - duty cycle too high ({duty_cycle:.2f})")
                continue
                
            print(f"   Period: {period_us:.0f}μs, Duty: {duty_cycle:.3f}")
            
            # Setup PWM
            pwm_out = pwm_pin_by_id(pwm_pin, frequency_hz=freq_hz)
            pwm_out.start(duty_cycle)
            time.sleep(0.5)
            
            # Test PIO pulse input only (faster test)
            pulse_pio = pulse_in_pin_by_id(pulse_pin, maxlen=32, 
                                         auto_clear=True, use_pio=True)
            pulse_pio.start(maxlen=32, auto_clear=True)
            
            measurements = []
            for i in range(5):
                pulses = pulse_pio.read_pulses()
                if pulses:
                    high_pulses = [p for p in pulses if p > 500]
                    measurements.extend(high_pulses)
                time.sleep(0.2)
            
            pulse_pio.stop()
            pwm_out.stop()
            
            if measurements:
                avg_measured = sum(measurements) / len(measurements)
                error = abs(avg_measured - target_pulse_us)
                error_pct = (error / target_pulse_us) * 100
                print(f"   Measured: {avg_measured:.1f}μs")
                print(f"   Error: {error:.1f}μs ({error_pct:.1f}%)")
            else:
                print("   No measurements")
                
    except Exception as e:
        print(f"Frequency test error: {e}")
    finally:
        if pwm_out:
            try:
                pwm_out.stop()
            except:
                pass

def test_timing_stability():
    """
    Test timing stability over longer period
    """
    print("\n" + "=" * 50) 
    print("Testing Timing Stability (30 seconds)")
    print("=" * 50)
    
    pwm_pin = "PICO.BCM.0"
    pulse_pin = "PICO.BCM.2"
    target_us = 1500
    
    try:
        # Setup stable 1500μs PWM
        pwm_out = pwm_pin_by_id(pwm_pin, frequency_hz=60)
        pwm_out.start(0.09)  # 1500μs
        
        pulse_pio = pulse_in_pin_by_id(pulse_pin, maxlen=64, 
                                     auto_clear=True, use_pio=True)
        pulse_pio.start(maxlen=64, auto_clear=True)
        
        measurements = []
        start_time = time.time()
        
        print("Collecting measurements for 30 seconds...")
        while time.time() - start_time < 30:
            pulses = pulse_pio.read_pulses()
            if pulses:
                high_pulses = [p for p in pulses if 1000 < p < 2000]
                measurements.extend(high_pulses)
                if len(measurements) % 50 == 0:  # Progress indicator
                    current_avg = sum(measurements[-10:]) / min(10, len(measurements))
                    print(f"   {len(measurements)} samples, recent avg: {current_avg:.1f}μs")
            time.sleep(0.1)
        
        pulse_pio.stop()
        pwm_out.stop()
        
        if measurements:
            avg = sum(measurements) / len(measurements)
            std_dev = (sum((x - avg)**2 for x in measurements) / len(measurements))**0.5
            min_val = min(measurements)
            max_val = max(measurements)
            
            print(f"\nStability Results:")
            print(f"   Total samples: {len(measurements)}")
            print(f"   Average: {avg:.2f}μs")
            print(f"   Std deviation: {std_dev:.2f}μs")
            print(f"   Range: {min_val:.1f} - {max_val:.1f}μs")
            print(f"   Error from target: {abs(avg - target_us):.2f}μs")
        
    except Exception as e:
        print(f"Stability test error: {e}")

if __name__ == "__main__":
    print("HARDWARE CONNECTION REQUIRED:")
    print("Connect jumper wire between GP0 (pin 1) and GP2 (pin 4)")
    print("Press Enter to continue or Ctrl+C to cancel...")
    
    try:
        input()
        test_pulse_accuracy()
        test_frequency_response() 
        test_timing_stability()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        print("\nTest completed - remove jumper wire")