import serial
import json
import logging

logger = logging.getLogger(__name__)


class Pico:
    """
    Pi Pico part to communicate with the Pico over usb. We use the same usb
    cable for power and data. The Pico is connected to the computer and can
    handle file copy / repl on /dev/ttyACM0 while doing bidirectional data
    transfer on /dev/ttyACM1.

    To set up the pins on the pico, we need to send a configuration dictionary.
    An example is here:

    setup_dict = {
        'input_pins': {
            'pin_13': dict(gpio='GP13', mode='INPUT', pull_up=False),
            'pulse_in': dict(gpio='GP16', mode='PULSE_IN', maxlen=4),
            'an_in': dict(gpio='GP28', mode='ANALOG_IN'),
        },
        'output_pins': {
            'pin_14': dict(gpio='GP14', mode='OUTPUT', value=0),
            'pwm_out': dict(gpio='GP15', mode='PWM',
                            frequency=60, duty_cycle=0.06),
        },
    }

    The dictionary is required to have either 'input_pins' or 'output_pins'
    or both as keys. Input and output refers to the pins on the pico,
    i.e. an input pin reads values and sends those as outputs of the Pico
    Donkeycar part. Don't get confused by that:
        Pi Pico input pin -> Donkeycar Pico part output
        Pi Pico output pin -> Donkeycar Pico part input

    The values of 'input_pins' and 'output_pins' contain dictionaries with
    the pin name as key and the pin configuration as value. The pin
    configuration is a dictionary with key-values depending on the mode of
    the pin. All pins use the 'gpio' key for the pin number. The 'mode' key
    is required for all pins. The different supported modes are:
        'INPUT': for digital input
        'PULSE_IN': for pulse input
        'ANALOG_IN': for analog input
    The different output modes are:
        'OUTPUT': for digital output
        'PWM': for pulse width modulation output
    See above examples for the required keys for each mode.
    """

    def __init__(self, port: str = '/dev/ttyACM1',
                 pin_configuration: dict = {},
                 send_config: bool = True):
        """
        Initialize the Pico part.
        :param port:                port for data connection
        :param pin_configuration:   configuration for the pins (see above)
        :param send_config:
        """
        self.serial = serial.Serial(port, 115200)
        self.counter = 0
        self.running = True
        self.pin_configuration = pin_configuration
        self.send_dict = dict()
        self.receive_dict = dict()
        self.send_config = send_config
        self.num_inputs = len(pin_configuration.get('input_pins', {}))
        logger.info(f"Pico added on port: {port}")

    def update(self):
        """
        Donkey parts interface. We are sending newline delimited json strings
        and expect the same in return.
        """
        if self.send_config:
            pack = json.dumps(self.pin_configuration) + '\n'
            self.serial.write(pack.encode())
        while self.running:
            pack = json.dumps(self.send_dict) + '\n'
            self.serial.write(pack.encode())
            # only read if there is something to read
            if self.serial.in_waiting == 0:
                continue
            bytes_in = self.serial.read_until()
            str_in = bytes_in.decode()[:-1]
            if self.counter % 1000 == 0:
                logger.debug(f'Last received: {bytes_in.decode()[:-1]}')
            try:
                received_dict = json.loads(str_in)
                self.receive_dict.update(received_dict)
            except ValueError as e:
                logger.error(f'Failed to load json because of {e}. Expected'
                             f' json, but got: +++{str_in}+++')
            self.counter += 1

    def run_threaded(self, *inputs):
        """
        Donkey parts interface
        """
        assert len(inputs) == self.num_inputs, \
            f"Expected {self.num_inputs} inputs but received {len(inputs)}"
        for k in self.send_dict.keys():
            self.send_dict[k] = inputs[k]
        return self.receive_dict.values()

    def shutdown(self):
        """
        Donkey parts interface
        """
        self.running = False
        logger.info(f"Pico disconnected, ran {self.counter} loops")
