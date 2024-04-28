import board  # type: ignore
import digitalio
import pulseio
import pwmio
import usb_cdc
import json


def input_pin_from_dict(d):
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
        pin = pulseio.PulseIn(gpio, maxlen=d.get('maxlen', 2))
        print(f'Configured pulse-in pin, gpio: {gpio}, maxlen: {pin.maxlen}')
    return pin


def output_pin_from_dict(d):
    gpio = getattr(board, d['gpio'])
    assert gpio != board.LED, 'Cannot use LED pin as output'
    pin = None
    if d['mode'] == 'OUTPUT':
        pin = digitalio.DigitalInOut(gpio)
        pin.direction = digitalio.Direction.OUTPUT
        pin.value = False
        print(f'Configured digital output pin, gpio: {gpio},', 
              f'value: {pin.value}')
    elif d['mode'] == 'PWM':
        duty_cycle_int = int(d.get('duty_cycle', 0.09) * 65535)
        freq = int(d.get('frequency', 60))
        pin = pwmio.PWMOut(gpio, frequency=freq)
        pin.duty_cycle = duty_cycle_int
        print(f'Configured pwm output pin, gpio: {gpio},',  
              f'frequency: {pin.frequency},', 
              f'duty_cycle: {pin.duty_cycle/65535}')
    return pin


def setup(str_in, input_pins, output_pins):
    for p in input_pins:
        p.deinit()
    for p in output_pins:
        p.deinit()
    input_pins.clear()
    output_pins.clear()
    d = dict()
    try:
        d = json.loads(str_in)
    except ValueError as e:
        print(f'Setup failed because of {e}. ' + 
              f'Expected json, but got: {str_in}')
        return False
    print(f'Received setup dict: {d}')
    for input_pin in d.get('input_pins', []):
        try:
            pin = input_pin_from_dict(input_pin)
            input_pins.append(pin)
        except Exception as e:
            print(f'Setup failed because of {e}.')
    for output_pin in d.get('output_pins', []):
        try:
            pin = output_pin_from_dict(output_pin)
            output_pins.append(pin)
        except Exception as e:
            print(f'Setup failed because of {e}.')
    return True


def update_output_pins(str_in, output_pins):
    try:
        float_list = [float(x) for x in str_in.strip('[]').split(', ')]
    except ValueError as e:
        print(f'Could not update output pins because of {e}.', 
              f'Expected list of floats, but got: {str_in}')
        float_list = []
    assert len(float_list) == len(output_pins), \
        'Number of values does not match number of output pins. '\
        f'str_in: {str_in}, float_list: {float_list}, out_pins: {output_pins}'
    for pin, value in zip(output_pins, float_list):
        try:
            if isinstance(pin, digitalio.DigitalInOut):
                pin.value = bool(value)
            elif isinstance(pin, pwmio.PWMOut):
                pin.duty_cycle = int(value * 65535)
            else:
                print(f'Cannot update pin {pin} with value {value}')
        except ValueError as e:
            print(f'Failed update pin {pin} with value {value} because of {e}')


def read(serial, input_pins, output_pins, led, is_setup):
    res = is_setup
    wait = serial.in_waiting
    if wait > 0:
        led.value = True
        bytes_in = serial.readline()
        str_in = bytes_in.decode()[:-1]
        serial.reset_input_buffer()
        # because we have timeout, str_in can be empty
        if str_in and str_in[0] == '[' and is_setup:
            update_output_pins(str_in, output_pins)
        elif str_in and str_in[0] == '{':
            res = setup(str_in, input_pins, output_pins)
    else:
        led.value = False
    return res


def write(serial, input_pins):
    """ Return list if no error or return error as string"""
    out_list = []
    for p in input_pins:
        if type(p) is digitalio.DigitalInOut:
            out_list.append(p.value)
        elif type(p) is pulseio.PulseIn:
            plist = [p[i] for i in range(len(p))]
            out_list.append(list(plist))

    out_string = str(out_list)
    out_string += '\n' 
    byte_out = out_string.encode()      
    n = serial.write(byte_out)
    return n


def main():
    serial = usb_cdc.data
    serial.timeout = 0.01
    serial.reset_input_buffer()
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT
    led.value = False
    count = 0
    is_setup = False
    input_pins = []
    output_pins = []
    try:
        while True:
            # reading input
            is_setup = read(serial, input_pins, output_pins, led, is_setup)
            # sending output, catching number of bytes written
            if is_setup:
                n = write(serial, input_pins)
            count += 1
    except KeyboardInterrupt:
        led.value = False   


if __name__ == '__main__':
    main()


