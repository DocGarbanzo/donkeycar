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


class PWMOutStraightThrough:
    def __init__(self, gpio, frequency=60, duty_cycle=0.09,
                 straight_input_pin=None, **kwargs):
        self.pin = pwmio.PWMOut(pin=gpio, frequency=frequency, **kwargs)
        self.pin.duty_cycle = int(duty_cycle * 65535)
        self.straight_input_pin = straight_input_pin
        if self.straight_input_pin:
            assert type(self.straight_input_pin) == PulseInResettable,\
                f'Straight-through routing needs PulseInResettable pin'
            assert not self.straight_input_pin.auto_clear, \
                ('Straight-through routing requires pulse-in pin to be '
                 'non-auto-clear')

    def deinit(self):
        self.pin.deinit()

    def set_duty_cycle(self, value):
        if isinstance(value, (float, int)):
            self.pin.duty_cycle = int(value * 65535)
        elif isinstance(value, str):
            r = self.straight_input_pin.get_readings()
            if len(r) > 1:
                self.pin.duty_cycle = int(min(r[-2:]) / sum(r[-2:]) * 65535)


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
        duty_cycle_int = int(d.get('duty_cycle', 0.09) * 65535)
        freq = int(d.get('frequency', 60))
        straight = d.get('straight')
        straight_input_pin = input_pins[straight] if straight else None
        pin = PWMOutStraightThrough(gpio, frequency=freq,
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
        elif type(pin) is PulseInResettable:
            write_dict[name] = pin.get_readings()

    byte_out = dict_to_bytes(write_dict)
    n = serial.write(byte_out)
    return n


def main():
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
    setup_dict = read_dict_from_nvm()
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

