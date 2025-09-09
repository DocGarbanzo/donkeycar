import time
import board
import digitalio
import analogio
import pulseio
import pwmio
import usb_cdc
import json
import struct
import microcontroller
import rp2pio
import adafruit_pioasm

class PulseInResettable:
    def __init__(self, gpio, maxlen=2, auto_clear=False, **kwargs):
        self.pin = pulseio.PulseIn(pin=gpio, maxlen=maxlen, **kwargs)
        self.auto_clear = auto_clear
        self.pin.clear()

    def get_value(self):
        l = len(self.pin)
        res = list(self.pin[i] for i in range(l))
        if self.auto_clear:
            self.pin.clear()
        return res

    def deinit(self):
        self.pin.deinit()


class PulseInPioPIO:
    def __init__(self, gpio, maxlen=64, auto_clear=False, frequency=2_000_000, **kwargs):
        import rp2pio
        import adafruit_pioasm
        import struct
        
        self.gpio = gpio
        self.maxlen = maxlen
        self.auto_clear = auto_clear
        self.buffer = bytearray(maxlen * 4)
        self.frequency = frequency
        # Calculate microsecond scaling factor
        # PIO loop takes 2 instructions per count, so effective frequency is frequency/2
        self.us_per_count = 2_000_000.0 / frequency
        
        program = """
.program pulse_both_us_nb
; Measure both high and low durations in clock counts
; Resolution depends on SM clock frequency
; Pushes one 32-bit value per level period
; Uses 'push noblock' so SM never blocks

wait_any:
    ; Sync to current state
    jmp pin, measure_high   ; if currently high, next edge is falling
    jmp      measure_low    ; else low, next edge is rising

; -----------------
; Measure HIGH width (from rising→falling)  
; -----------------
measure_high:
    mov  x, ~x              ; x = -1 (0xFFFFFFFF)
count_high:
    jmp  pin, dec_high      ; if still high, loop
    ; fell low: push high duration
    mov  isr, x
    push noblock
    jmp  measure_low        ; now measure low
dec_high:
    jmp  x--, count_high    ; decrement and loop (2 cycles per iteration)

; -----------------
; Measure LOW width (from falling→rising)
; -----------------
measure_low:
    mov  x, ~x              ; x = -1 (0xFFFFFFFF)
count_low:
    jmp  pin, low_done      ; if high now, edge occurred
    jmp  x--, count_low     ; still low, keep counting (2 cycles per iteration)
low_done:
    mov  isr, x
    push noblock
    jmp  measure_high
"""
        
        asm = adafruit_pioasm.assemble(program)
        
        try:
            self.sm = rp2pio.StateMachine(
                asm,
                frequency=frequency,
                first_in_pin=gpio,
                in_pin_count=1,
                auto_push=False,
                fifo_join=rp2pio.StateMachine.RX_FIFO_JOIN
            )
            # Verify actual frequency matches requested
            actual_freq = self.sm.frequency
            if abs(actual_freq - frequency) > frequency * 0.01:  # 1% tolerance
                print(f"Warning: Requested frequency {frequency}Hz, actual {actual_freq}Hz")
                # Update scaling factor with actual frequency
                self.us_per_count = 2_000_000.0 / actual_freq
        except Exception as e:
            print(f"PIO StateMachine setup failed: {e}")
            raise

    def get_value(self):

        pulses = []
        nbytes = self.sm.readinto(self.buffer)
        for i in range(0, nbytes, 4):
            (xval,) = struct.unpack_from("<I", self.buffer, i)
            # Convert inverted count to actual count
            count = (~xval) & 0xFFFFFFFF
            # Scale to microseconds based on actual frequency
            us = count * self.us_per_count
            pulses.append(int(us))
        return pulses

    def deinit(self):
        if hasattr(self, 'sm'):
            self.sm.deinit()


class PWMIn(PulseInResettable):
    def __init__(self, gpio, frequency=60, duty=0.09, min_us=1000,
                 max_us=2000, **kwargs):
        super().__init__(gpio, maxlen=2, auto_clear=False, **kwargs)
        self.duty = duty
        self.min_us = min_us
        self.max_us = max_us
        self.frequency = frequency

    def get_value(self):
        """
        Get the duty cycle from the last two readings. Assuming min and max
        us are 1000 and 2000, respectively,
        """
        r = super().get_value()
        if len(r) > 1:
            duty_us = min(r[-2], r[-1])
            # High signals should be between min_us and max_us. As we
            # occasionally see duplicated readings like [1000, 1000] instead
            # of [15666, 1000] we ignore readings which are out by more than
            # 10% of the min_us or max_us.

            if duty_us < 0.9 * self.min_us or duty_us > 1.1 * self.max_us:
                return self.duty
            self.duty = duty_us * self.frequency * 1e-6
        return self.duty


class PWMOut:
    def __init__(self, gpio, frequency=60, duty_cycle=0.09, **kwargs):
        self.pin = pwmio.PWMOut(pin=gpio, frequency=frequency, **kwargs)
        self.pin.duty_cycle = int(duty_cycle * 65535)

    def deinit(self):
        self.pin.deinit()

    def set_value(self, value):
        """ Set the duty cycle of the PWM output. """
        self.pin.duty_cycle = int(value * 65535)


class DigitalIO:
    def __init__(self, gpio, direction: digitalio.Direction, pull: int = None,
                 value=0):
        self.pin = digitalio.DigitalInOut(gpio)
        self.pin.direction = direction
        # for input pins, set pull-up or pull-down
        if direction == digitalio.Direction.INPUT and pull:
            if pull == 1:
                self.pin.pull = digitalio.Pull.UP
            elif pull == 2:
                self.pin.pull = digitalio.Pull.DOWN
        self.set_value(value)

    def set_value(self, value):
        if self.pin.direction == digitalio.Direction.OUTPUT:
            self.pin.value = value

    def get_value(self):
        return self.pin.value

    def deinit(self):
        self.pin.deinit()


class AnalogIN:
    def __init__(self, gpio):
        self.pin = analogio.AnalogIn(gpio)

    def get_value(self):
        return self.pin.value

    def deinit(self):
        self.pin.deinit()


def bytes_to_dict(byte_data, count):
    if byte_data == b'':
        return {}
    str_in = byte_data.decode()[:-1]
    if not str_in:
        return {}
    try:
        out_dict = json.loads(str_in)
        return out_dict
    except ValueError as e:
        print(f'Failed to decode JSON because of {e}',
              f'from {str_in} in loop {count}.')
    return {}


def dict_to_bytes(dict_data):
    str_out = json.dumps(dict_data) + '\n'
    byte_out = str_out.encode()
    return byte_out


def pin_from_dict(pin_name, d):
    print(f'Creating pin {pin_name} from dict: {d}')
    # convert from pin_name string to board pin object
    gpio = getattr(board, pin_name)
    assert gpio != board.LED, 'Cannot assign LED pin as input or output.'
    pin = None
    if d['mode'] == 'INPUT':
        pin = DigitalIO(gpio, digitalio.Direction.INPUT, d.get('pull'))
        print(f'Configured digital input pin, gpio: {gpio}, pull:',
              f'{pin.pin.pull}')
    elif d['mode'] == 'PULSE_IN':
        pin = PulseInResettable(gpio, maxlen=d.get('maxlen', 2),
                                auto_clear=d.get('auto_clear', False))
        print(f'Configured pulse-in pin, gpio: {gpio}, maxlen:',
              f'{pin.pin.maxlen}, auto_clear: {pin.auto_clear}')
    elif d['mode'] == 'PULSE_IN_PIO':
        frequency = d.get('frequency', 2_000_000)
        pin = PulseInPioPIO(gpio, maxlen=d.get('maxlen', 64),
                            auto_clear=d.get('auto_clear', False),
                            frequency=frequency)
        print(f'Configured PIO pulse-in pin, gpio: {gpio}, maxlen: {pin.maxlen}',
              f'auto_clear: {pin.auto_clear}, frequency: {pin.frequency}Hz')
    elif d['mode'] == 'PWM_IN':
        pin = PWMIn(gpio, duty=d.get('duty_center', 0.09))
        print(f'Configured pwm-in pin, gpio: {gpio}, duty_center: {pin.duty}')
    elif d['mode'] == 'ANALOG_IN':
        pin = AnalogIN(gpio)
        print(f'Configured analog input pin, gpio: {gpio}')
    elif d['mode'] == 'OUTPUT':
        pin = DigitalIO(gpio, digitalio.Direction.OUTPUT)
        print(f'Configured digital output pin, gpio: {gpio},',
              f'value: {pin.pin.value}')
    elif d['mode'] == 'PWM':
        duty_cycle = d.get('duty_cycle', 0.09)
        freq = int(d.get('frequency', 60))
        pin = PWMOut(gpio, frequency=freq, duty_cycle=duty_cycle)
        print(f'Configured pwm output pin, gpio: {gpio},',
              f'frequency: {pin.pin.frequency},',
              f'duty_cycle: {pin.pin.duty_cycle / 65535}')
    return pin


def deinit_pin(pin):
    try:
        pin.deinit()
        print(f'De-initialise pin: {pin}')
    except AttributeError as e:
        print(f'Pin has no deinit method: {e}')
    except Exception as e:
        print(f'Pin deinit failed: {e}')


def reset_all_pins(input_pins, output_pins):
    for pin in (input_pins | output_pins).values():
        deinit_pin(pin)
    input_pins.clear()
    output_pins.clear()
    print(f'Reset all pins.')


def setup(setup_dict, input_pins, output_pins):
    if not setup_dict:
        return False
    # if both input_pins and output_pins are empty, we are clearing all pins
    print(f'Starting setup -------->')
    print(f'Received setup dict: {setup_dict}')
    in_dict, out_dict = tuple(setup_dict.get(key, {})
                              for key in ['input_pins', 'output_pins'])
    if len(in_dict) == 0 and len(out_dict) == 0:
        reset_all_pins(input_pins, output_pins)
    # merge pins from setup dict into input_pins and output_pins
    t_list = zip((in_dict, out_dict), (input_pins, output_pins))
    for setup_io_dict, pins in t_list:
        for pin_name, pin_dict in setup_io_dict.items():
            if pin_name in pins:
                deinit_pin(pins[pin_name])
                if len(pin_dict) == 0:
                    print(f'Removing pin {pin_name}')
                    del pins[pin_name]
                    continue
                else:
                    print(f'Overwriting {pin_name}')
            try:
                pins[pin_name] = pin_from_dict(pin_name, pin_dict)
            except Exception as e:
                print(f'Setup of {pin_name} failed because of {e}.')
                print(f'Finished setup unexpectedly <--------')
                return False

    print(f'Updated input pins: {input_pins}')
    print(f'Updated output pins: {output_pins}')
    print(f'Finished setup <--------')
    return True


def update_output_pins(output_data, output_pins, led):
    if output_data:
        led.value = True
    for pin_name, value in output_data.items():
        out_pin = output_pins.get(pin_name)
        if out_pin:
            try:
                out_pin.set_value(value)
            except ValueError as e:
                print(f'Failed updating output pin {pin_name} because of {e}')
    if output_data:
        led.value = False


def read(serial, input_pins, output_pins, led, is_setup, count):

    bytes_in = serial.readline()
    read_dict = bytes_to_dict(bytes_in, count)
    # if setup dict sent, this contains 'input_pins' or 'output_pins'
    if 'input_pins' in read_dict or 'output_pins' in read_dict:
        is_setup = setup(read_dict, input_pins, output_pins)
    # only call update_output_pins if setup has been done
    elif is_setup:
        update_output_pins(read_dict, output_pins, led)
    return is_setup


def write(serial, input_pins, write_dict, led):
    """ Return list if no error or return error as string"""
    if input_pins:
        led.value = True
    for name, pin in input_pins.items():
        write_dict[name] = pin.get_value()

    byte_out = dict_to_bytes(write_dict)
    n = serial.write(byte_out)
    if input_pins:
        led.value = False
    return n


def main():
    print('\n************ Starting pi pico ************')
    microcontroller.cpu.frequency = 180_000_000
    print(f'Current CPU frequency: {microcontroller.cpu.frequency}')

    serial = usb_cdc.data
    serial.timeout = 0.01
    serial.reset_input_buffer()
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT
    led.value = False

    input_pins = {}
    output_pins = {}
    write_dict = {}
    is_setup = False
    count = 0
    tic = time.monotonic()
    total_time = 0
    try:
        while True:
            # reading input
            is_setup = read(serial, input_pins, output_pins, led,
                            is_setup, count)
            # sending output, catching number of bytes written
            if is_setup:
                n = write(serial, input_pins, write_dict, led)
            toc = time.monotonic()
            total_time += toc - tic
            tic = toc
            count += 1
    except KeyboardInterrupt:
        led.value = False


if __name__ == '__main__':
    main()
