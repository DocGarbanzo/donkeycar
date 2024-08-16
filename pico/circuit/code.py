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
    def __init__(self, gpio, duty=0.09, **kwargs):
        super().__init__(gpio, maxlen=2, auto_clear=False, **kwargs)
        self.duty = duty

    def get_readings(self):
        r = super().get_readings()
        if len(r) > 1:
            this_duty = min(r[-2], r[-1]) / (r[-2] + r[-1])
            # Duty cycle is typically 6% - 12%. A more abrupt change than 2%
            # is likely to be some form of corrupt signal and will be ignored
            if abs(this_duty - self.duty) < 0.02:
                self.duty = this_duty
            else:
                print(f'PWMIn duty change too abrupt: {this_duty-self.duty}',
                      f'pulse readings {r}')
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
        self.pin.duty_cycle = int(value * 65535)


def write_bytes_to_nvm(byte_data):
    # first clear the memory
    l = len(byte_data)
    if l >= 10000:
        print(f'Cannot write {l} bytes to nvm, too large.')
    bytes_to_save = f'{l:04}'.encode() + byte_data
    microcontroller.nvm[0:l+4] = bytes_to_save
    print(f'Wrote {l} bytes to nvm.')


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


def read_dict_from_nvm():
    try:
        lb = microcontroller.nvm[0:4]
        lsb = lb.decode()
        if not lsb.isdigit():
            print(f'Failed to read length from nvm: {lsb}')
            return {}
        l = int(lsb)
        byte_data = microcontroller.nvm[4:l+4]  # Read from NVM
        dict_data = bytes_to_dict(byte_data, 0)
        print(f'Read setup dict from nvm: {dict_data}')
        return dict_data
    except Exception as e:
        print(f'Failed to read setup dict from nvm: {e}')
        return {}


def pin_from_dict(d, input_pins):
    gpio = getattr(board, d['gpio'])
    assert gpio != board.LED, 'Cannot use LED pin as input'
    pin = None
    if d['mode'] == 'INPUT':
        pin = digitalio.DigitalInOut(gpio)
        pin.direction = digitalio.Direction.INPUT
        pin.pull = digitalio.Pull.UP if d.get('pull_up', False) \
            else digitalio.Pull.DOWN
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


def setup(setup_dict, input_pins, output_pins, store=False):
    if not setup_dict:
        return False
    for pin in (input_pins | output_pins).values():
        try:
            pin.deinit()
            print(f'De-initialise pin: {pin}')
        except AttributeError as e:
            print(f'Pin has no deinit method: {e}')
        except Exception as e:
            print(f'Pin deinit failed: {e}')
    input_pins.clear()
    output_pins.clear()
    print(f'Received setup dict: {setup_dict}')
    t_list = zip([input_pins, output_pins], ['input_pins', 'output_pins'])
    for pins, pin_key in t_list:
        for pin_name, pin_dict in setup_dict.get(pin_key, {}).items():
            try:
                pins[pin_name] = pin_from_dict(pin_dict, input_pins)
            except Exception as e:
                print(f'Setup of {pin_name} failed because of {e}.')
                return False

    if store:
        byte_data = dict_to_bytes(setup_dict)
        write_bytes_to_nvm(byte_data)
    print(f'Created input pins: {input_pins}')
    print(f'Created output pins: {output_pins}')
    return True


def update_output_pins(output_data, input_pins, output_pins):
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
            is_setup = setup(read_dict, input_pins, output_pins, store=True)
        elif is_setup:
            update_output_pins(read_dict, input_pins, output_pins)
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
    # initialise input/output pins from nvm
    setup_dict = {} #read_dict_from_nvm()
    is_setup = setup(setup_dict, input_pins, output_pins, store=False)
    print(f'Successful setup from nvm: {is_setup}.')
    write_dict = {}
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

