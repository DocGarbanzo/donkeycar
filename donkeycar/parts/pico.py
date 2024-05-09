import time

import serial
import json
import logging
from threading import Lock

logger = logging.getLogger(__name__)


class Pico:
    """
    Pi Pico part to communicate with the Pico over usb. We use the same usb
    cable for power and data. The Pico is connected to the computer and can
    handle file copy / repl on /dev/ttyACM0 while doing bidirectional data
    transfer on /dev/ttyACM1.

    To set up the pins on the pico, we need to send a configuration dictionary.
    An example is here:

    pin_configuration = {
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

    The Pi Pico stores the pin configuration in non-volatile memory (NVM) and
    reloads it on startup. Therefore, it is not necessary to send the pin
    configuration every time the Pico is started. As long as the pin
    configuration is not changed there is no need to every send the
    configuration a second time.


    """

    def __init__(self, port: str = '/dev/ttyACM1',
                 pin_configuration: dict = {},
                 send_config: bool = True):
        """
        Initialize the Pico part.
        :param port:                port for data connection
        :param pin_configuration:   configuration for the pins (see above)
        :param send_config:         if the configuration should be sent to
                                    the Pico at startup
        """
        self.serial = serial.Serial(port, 115200)
        self.counter = 0
        self.running = True
        self.pin_configuration = pin_configuration
        self.send_keys = pin_configuration.get('output_pins', {}).keys()
        self.send_dict = dict()
        self.receive_dict = dict()
        self.send_config = send_config
        self.lock = Lock()
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
            self.lock.acquire()
            pack = json.dumps(self.send_dict) + '\n'
            self.lock.release()
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
                self.lock.acquire()
                self.receive_dict.update(received_dict)
                self.lock.release()
            except ValueError as e:
                logger.error(f'Failed to load json in loop {self.counter} '
                             f'because of {e}. Expected json, but got: '
                             f'+++{str_in}+++')
            time.sleep(0)
            self.counter += 1

    def run_threaded(self, *inputs):
        """
        Donkey parts interface
        """
        self.lock.acquire()
        n = len(self.receive_dict)
        ret = list(self.receive_dict.values()) if n > 1 \
            else list(self.receive_dict.values())[0] if n == 1 else None
        # allow receiving no data and just returning the current state
        if not inputs:
            self.lock.release()
            return ret
        assert len(inputs) == len(self.send_keys), \
            f"Expected {len(self.send_keys)} inputs but received {len(inputs)}"
        for key, inp in zip(self.send_keys, inputs):
            self.send_dict[key] = inp
        self.lock.release()
        return ret

    def run(self, *inputs):
        """
        Donkey parts interface. Allow adding as non-threaded for multiple use.
        """
        return self.run_threaded(*inputs)

    def shutdown(self):
        """
        Donkey parts interface
        """
        self.running = False
        logger.info(f"Pico disconnected, ran {self.counter} loops")


class PicoPWMOutput:
    """
    Donkey part to convert an input float number into a pwm duty cycle output
    for the Pico's PWMOut pins. The input is a float in [in_min, in_max]
    mapping to a duty cycle float within [duty_min, duty_max].
    Note you can provide duty_max < duty_min, if an inversion of the signal
    is required. Standard RC ESC and sero signals range between 1000us and
    2000us, with 1500us being the center. At a 60Hz frequency this translates
    to duty cycles between 6%-12%.
    """

    def __init__(self, in_min=0.0, in_max=1.0, duty_min=0.06, duty_max=0.012,
                 round_output_digits=3):
        """
        Initialize the PicoPWMOut part.
        :param in_min:      minimum input value
        :param in_max:      maximum input value
        :param duty_min:    minimum duty cycle
        :param duty_max:    maximum duty cycle
        """
        self.in_min = in_min
        self.in_max = in_max
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.round_output_digits = round_output_digits
        logger.info(f"PicoPWMOut created with min:{in_min}, max:{in_max}, " 
                    f"duty min:{duty_min}, duty max:{duty_max}, round digits: "
                    f"{round_output_digits}")

    def run(self, x):
        res = (self.duty_min + (x - self.in_min)
               * (self.duty_max - self.duty_min) / (self.in_max - self.in_min))
        return round(res, self.round_output_digits)

    def shutdown(self):
        pass


class PicoPWMInput:
    """
    Donkey part to convert a PulseIn signal from the Pico into a float output
    between out min and out max. The PulseIn signal is expected to be a list
    of integers containing hi, lo signals in microseconds.

    """
    def __init__(self, out_min=0.0, out_max=1.0, out_center=None,
                 duty_min=0.06, duty_max=0.012):

        """
        Initialize the PicoPWMInput part.
        :param out_min:     minimum output value
        :param out_max:     maximum output value
        :param out_center:      center value
        :param duty_min:    minimum duty cycle
        :param duty_max:    maximum duty cycle
        """
        self.out_min = out_min
        self.out_max = out_max
        self.out_center = out_center or (out_max + out_min) / 2
        self.duty_min = duty_min
        self.duty_max = duty_max
        logger.info(
            f"PicoPWMInput created with min:{out_min} and max:{out_max} and "
            f"center:{self.out_center}")

    def run(self, pulse_in):
        # most recent measurements will be in the last 2 entries
        if pulse_in and len(pulse_in) > 2:
            duty = min(pulse_in[-2:]) / sum(pulse_in[-2:])
            duty_rel = (duty - self.duty_min) / (self.duty_max - self.duty_min)
            res = self.out_min + duty_rel * (self.out_max - self.out_min)
            return res
        return self.out_center
