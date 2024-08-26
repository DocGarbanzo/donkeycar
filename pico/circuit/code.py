import time
import board
import digitalio
import analogio
import pulseio
import pwmio
import usb_cdc
import json
import microcontroller


class PulseInResettable:
    def __init__(self, gpio, maxlen=2, auto_clear=False, **kwargs):
        self.pin = pulseio.PulseIn(pin=gpio, maxlen=maxlen, **kwargs)
        self.auto_clear = auto_clear
        self.pin.clear()

    def get_readings(self):
        l = len(self.pin)
        res = list(self.pin[i] for i in range(l))
        if self.auto_clear:
            self.pin.clear()
        return res

    def deinit(self):
        self.pin.deinit()


class PWMIn(PulseInResettable):
    def __init__(self, gpio, frequency=60, duty=0.09, min_us=1000,
                 max_us=2000, **kwargs):
        super().__init__(gpio, maxlen=2, auto_clear=False, **kwargs)
        self.duty = duty
        self.min_us = min_us
        self.max_us = max_us
        self.frequency = frequency

    def get_readings(self):
        """
        Get the duty cycle from the last two readings. Assuming min and max
        us are 1000 and 2000, respectively,
        """
        r = super().get_readings()
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


class PWMOutStraightThrough:
    def __init__(self, gpio, frequency=60, duty_cycle=0.09,
                 straight_input_pin=None, **kwargs):
        self.pin = pwmio.PWMOut(pin=gpio, frequency=frequency, **kwargs)
        self.pin.duty_cycle = int(duty_cycle * 65535)
        self.straight_input_pin = straight_input_pin
        if self.straight_input_pin:
            s_type = type(self.straight_input_pin)
            assert s_type == PWMIn, \
                (f'Straight-through output pin needs PWMIn pin but found type {s_type}')

    def deinit(self):
        self.pin.deinit()

    def set_duty_cycle(self, value):
        """ Set the duty cycle of the PWM output
        If value is positive, set the duty cycle to that value,
        otherwise use the straight input pin to set the duty cycle.
        """
        v = value if value > 0 else self.straight_input_pin.get_readings()
        self.pin.duty_cycle = int(v * 65535)


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


def pin_from_dict(pin_name, d, input_pins):
    print(f'Creating pin from dict: {d}')
    # convert from pin_name string to board pin object
    gpio = getattr(board, pin_name)
    assert gpio != board.LED, 'Cannot use LED pin as input'
    pin = None
    if d['mode'] == 'INPUT':
        pin = digitalio.DigitalInOut(gpio)
        pin.direction = digitalio.Direction.INPUT
        pull = d.get('pull')  # None will work here otherwise map
        if pull == 1:
            pull = digitalio.Pull.UP
        elif pull == 2:
            pull = digitalio.Pull.DOWN
        pin.pull = pull
        print(f'Configured digital input pin, gpio: {gpio}, pull: {pin.pull}')
    elif d['mode'] == 'PULSE_IN':
        pin = PulseInResettable(gpio, maxlen=d.get('maxlen', 2),
                                auto_clear=d.get('auto_clear', False))
        print(f'Configured pulse-in pin, gpio: {gpio}, maxlen:',
              f'{pin.pin.maxlen}, auto_clear: {pin.auto_clear}')
    elif d['mode'] == 'PWM_IN':
        pin = PWMIn(gpio, duty=d.get('duty_center', 0.09))
        print(f'Configured pwm-in pin, gpio: {gpio}, duty_center: {pin.duty}')
    elif d['mode'] == 'ANALOG_IN':
        pin = analogio.AnalogIn(gpio)
        print(f'Configured analog input pin, gpio: {gpio}')
    elif d['mode'] == 'OUTPUT':
        pin = digitalio.DigitalInOut(gpio)
        pin.direction = digitalio.Direction.OUTPUT
        pin.value = False
        print(f'Configured digital output pin, gpio: {gpio},',
              f'value: {pin.value}')
    elif d['mode'] == 'PWM':
        duty_cycle = d.get('duty_cycle', 0.09)
        freq = int(d.get('frequency', 60))
        straight = d.get('straight')
        straight_input_pin = input_pins[straight] if straight else None
        pin = PWMOutStraightThrough(gpio, frequency=freq,
                                    duty_cycle=duty_cycle,
                                    straight_input_pin=straight_input_pin)
        print(f'Configured pwm output pin, gpio: {gpio},',
              f'frequency: {pin.pin.frequency},',
              f'duty_cycle: {pin.pin.duty_cycle / 65535},',
              f'pwm output: {pin.pin}, straight: {pin.straight_input_pin}')
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
                print(f'Overwriting {pin_name}')
                deinit_pin(pins[pin_name])
            try:
                pins[pin_name] = pin_from_dict(pin_name, pin_dict, input_pins)
            except Exception as e:
                print(f'Setup of {pin_name} failed because of {e}.')
                return False

    print(f'Updated input pins: {input_pins}')
    print(f'Updated output pins: {output_pins}')
    return True


def update_output_pins(output_data, output_pins):
    for pin_name, value in output_data.items():
        out_pin = output_pins.get(pin_name)
        try:
            if isinstance(out_pin, digitalio.DigitalInOut):
                out_pin.value = bool(value)
            elif isinstance(out_pin, PWMOutStraightThrough):
                out_pin.set_duty_cycle(value)
            else:
                print(f'Cannot update pin out_pin: {pin_name} \
                      because of unknown type {type(out_pin)}.')
        except ValueError as e:
            print(f'Failed update output pin {pin_name} because of {e}')


def read(serial, input_pins, output_pins, led, is_setup, count):
    if serial.in_waiting > 0:
        led.value = True
        bytes_in = serial.readline()
        #serial.reset_input_buffer()
        read_dict = bytes_to_dict(bytes_in, count)
        # if setup dict sent, this contains 'input_pins' or 'output_pins'
        if 'input_pins' in read_dict or 'output_pins' in read_dict:
            is_setup = setup(read_dict, input_pins, output_pins)
        elif is_setup:
            update_output_pins(read_dict, output_pins)
    else:
        led.value = False
    return is_setup


def write(serial, input_pins, write_dict):
    """ Return list if no error or return error as string"""
    for name, pin in input_pins.items():
        if type(pin) in (digitalio.DigitalInOut, analogio.AnalogIn):
            write_dict[name] = pin.value
        elif type(pin) in (PulseInResettable, PWMIn):
            write_dict[name] = pin.get_readings()

    byte_out = dict_to_bytes(write_dict)
    n = serial.write(byte_out)
    return n


def main():
    print('\n************ Starting pi pico ************')
    microcontroller.cpu.frequency = 180000000
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
                n = write(serial, input_pins, write_dict)
            toc = time.monotonic()
            total_time += toc - tic
            tic = toc
            count += 1
    except KeyboardInterrupt:
        led.value = False


if __name__ == '__main__':
    main()

