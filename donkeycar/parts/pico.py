import time
from collections import deque
from statistics import fmean

import serial
import json
import logging
from threading import Lock, Thread

logger = logging.getLogger(__name__)


class PicoPart:
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
            'odo_in': dict(gpio='GP18', mode='PULSE_IN',
                           maxlen=8, auto_clear=True),
            'pulse_in': dict(gpio='GP16', mode='PULSE_IN', maxlen=4),
            'an_in': dict(gpio='GP28', mode='ANALOG_IN'),
        },
        'output_pins': {
            'pin_14': dict(gpio='GP14', mode='OUTPUT', value=0),
            'pwm_out': dict(gpio='GP15', mode='PWM', frequency=60,
                            duty_cycle=0.06, straight='pulse_in'),
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
        self.receive_keys = pin_configuration.get('input_pins', {}).keys()
        self.send_dict = dict()
        # need to guarantee dictionary order as we might receive the
        # dictionary with a different order of keys after deserialisation
        self.receive_dict = dict((k, None) for k in self.receive_keys)
        self.send_config = send_config
        self.lock = Lock()
        self.start = None
        logger.info(f"Pico created on port: {port}")

    def update(self):
        """
        Donkey parts interface. We are sending newline delimited json strings
        and expect the same in return.
        """
        if self.send_config:
            pack = json.dumps(self.pin_configuration) + '\n'
            self.serial.write(pack.encode())
        # clear the input buffer
        self.serial.reset_input_buffer()
        self.start = time.time()
        while self.running:
            try:
                self.lock.acquire()
                pack = json.dumps(self.send_dict) + '\n'
                self.lock.release()
                self.serial.write(pack.encode())
                bytes_in = self.serial.read_until()
                str_in = bytes_in.decode()[:-1]
                received_dict = json.loads(str_in)
                self.lock.acquire()
                self.receive_dict.update(received_dict)
                self.lock.release()
                if self.counter % 1000 == 0:
                    logger.debug(f'Last sent: {pack}')
                    logger.debug(f'Last received: {str_in}')
            except ValueError as e:
                logger.error(f'Failed to load json in loop {self.counter} '
                             f'because of {e}. Expected json, but got: '
                             f'+++{str_in}+++')
            except Exception as e:
                logger.error(f'Problem with serial input {e}')
            self.counter += 1

    def run_threaded(self, *inputs):
        """
        Donkey parts interface
        """
        # Wait until threaded loop has at least run once, so we don't have to
        # process None values. This blocks until the first data is received.
        while self.counter == 0:
            time.sleep(0.1)
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
        time.sleep(0.1)
        self.serial.close()
        total_time = time.time() - self.start
        logger.info(f"Pico part disconnected, ran {self.counter} loops, "
                    f"each loop taking "
                    f"{total_time * 1000 / self.counter:5.1f} ms.")


class Pico:
    """
    Pi Pico class to communicate with the Pico over usb. We use the same usb
    cable for power and data. The Pico is connected to the computer and can
    handle file copy / repl on /dev/ttyACM0 while doing bidirectional data
    transfer on /dev/ttyACM1.

    To set up the pins on the pico, we need to send a configuration dictionary.
    An example is here:

    pin_configuration = {
        'input_pins': {
            'pin_13': dict(gpio='GP13', mode='INPUT', pull_up=False),
            'odo_in': dict(gpio='GP18', mode='PULSE_IN',
                           maxlen=8, auto_clear=True),
            'pulse_in': dict(gpio='GP16', mode='PULSE_IN', maxlen=4),
            'an_in': dict(gpio='GP28', mode='ANALOG_IN'),
        },
        'output_pins': {
            'pin_14': dict(gpio='GP14', mode='OUTPUT', value=0),
            'pwm_out': dict(gpio='GP15', mode='PWM', frequency=60,
                            duty_cycle=0.06, straight='pulse_in'),
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

    def __init__(self, port: str = '/dev/ttyACM1'):
        """
        Initialize the Pico part.
        :param port:                port for data connection
        """
        self.serial = serial.Serial(port, 115200)
        self.counter = 0
        self.running = True
        self.pin_configuration = dict()
        self.send_dict = dict()
        # need to guarantee dictionary order as we might receive the
        # dictionary with a different order of keys after deserialisation
        self.receive_dict = dict()
        self.lock = Lock()
        self.start = None
        logger.info(f"Pico created on port: {port}")
        self.t = Thread(target=self.loop, args=())

    def loop(self):
        """
        Donkey parts interface. We are sending newline delimited json strings
        and expect the same in return.
        """
        # clear the input buffer
        self.serial.reset_input_buffer()
        self.start = time.time()
        # start loop of continuous communication
        while self.running:
            try:
                pack = None
                with self.lock:
                    pack = json.dumps(self.send_dict) + '\n'
                self.serial.write(pack.encode())
                time.sleep(0)
                bytes_in = self.serial.read_until()
                time.sleep(0)
                str_in = bytes_in.decode()[:-1]
                received_dict = json.loads(str_in)
                with self.lock:
                    self.receive_dict.update(received_dict)
                if self.counter % 1000 == 0:
                    logger.debug(f'Last sent: {pack}')
                    logger.debug(f'Last received: {str_in}')
            except ValueError as e:
                logger.error(f'Failed to load json in loop {self.counter} '
                             f'because of {e}. Expected json, but got: '
                             f'+++{str_in}+++')
            except Exception as e:
                logger.error(f'Problem with serial input {e}')
            self.counter += 1

    def write(self, gpio: str, value: float or int) -> None:
        """
        :param gpio:    the gpio pin to write to
        :param value:   the value to write
        """
        # Wait until threaded loop has at least run once, so we don't have to)
        # process None values. This blocks until the first data is received.
        while self.counter == 0:
            time.sleep(0.1)
        assert gpio in self.send_dict, f"Pin {gpio} not in send_dict."
        with self.lock:
            self.send_dict[gpio] = value

    def read(self, gpio):
        """
        :param gpio:    the gpio pin to read from
        :return:        the value of the pin
        """
        # Wait until threaded loop has at least run once, so we don't have to
        # process None values. This blocks until the first data is received.
        while self.counter == 0:
            time.sleep(0.1)
        assert gpio in self.receive_dict, f"Pin {gpio} not in receive_dict."
        with self.lock:
            return self.receive_dict[gpio]

    def stop(self):
        self.running = False
        time.sleep(0.1)
        self.t.join()
        self.serial.close()
        total_time = time.time() - self.start
        logger.info(f"Pico communication disconnected, ran {self.counter} "
                    f"loops, each loop taking "
                    f"{total_time * 1000 / self.counter:5.1f} ms.")

    def setup_input_pin(self, gpio: str, mode: str, **kwargs) -> None:
        """
        :param gpio:    the gpio pin to set up
        :param mode:    the mode of the pin
        :param kwargs:  additional arguments for the mode
        """
        logger.info(f"Setting up input pin {gpio} in mode {mode} with {kwargs}")
        assert mode in ['INPUT', 'PULSE_IN', 'ANALOG_IN'], \
            f"Mode {mode} not supported for input pins."
        setup_dict = dict(input_pins=dict(gpio=dict(mode=mode, **kwargs)))
        with self.lock:
            # send the setup dictionary
            pack = json.dumps(setup_dict) + '\n'
            self.serial.write(pack.encode())


instance = Pico()


class DutyScaler:
    """
    Donkey part to convert an input float number into a pwm duty cycle output
    for the Pico's PWMOut pins. The input is a float in [in_min, in_max]
    mapping to a duty cycle float within [duty_min, duty_max].
    Note you can provide duty_max < duty_min, if an inversion of the signal
    is required. Standard RC ESC and sero signals range between 1000us and
    2000us, with 1500us being the center. At a 60Hz frequency this translates
    to duty cycles between 6%-12%.
    """

    def __init__(self, x_min=0.0, x_max=1.0, x_center=None, x_deadband=None,
                 duty_min=0.06, duty_max=0.012, duty_center=None,
                 round_digits=3, to_duty=False):
        """
        Initialize the PicoPWMOut part.
        :param in_min:      minimum input value
        :param in_max:      maximum input value
        :param duty_min:    minimum duty cycle
        :param duty_max:    maximum duty cycle
        """
        self.x_min = x_min
        self.x_max = x_max
        self.x_center = x_center or (x_max + x_min) / 2
        self.x_deadband = x_deadband
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_center = duty_center or (duty_max + duty_min) / 2
        self.round_digits = round_digits
        self.to_duty = to_duty
        logger.info(f"DutyScaler "
                    f"{'to_duty' if to_duty else 'from_duty'} created with min"
                    f":{x_min}, max:{x_max}, duty min:{duty_min}, "
                    f"duty max:{duty_max}, round digits: {round_digits} and "
                    f"deadband: {x_deadband}")

    @staticmethod
    def bilinear_interpolate(x, x_min, x_max, x_c, y_min, y_max, y_c):
        """
        Bilinear interpolation for a point x between two points (x_min, y_min)
        and a center point x_c, such that for x_min <= x <= x_c we return
        values in [y_min, y_c] and for x_c <= x <= x_max we return values in
        [y_c, y_max].
        """
        if x is None:
            return y_c
        if x < x_min:
            return y_min
        if x > x_max:
            return y_max
        if x < x_c:
            return y_min + (x - x_min) * (y_c - y_min) / (x_c - x_min)
        return y_c + (x - x_c) * (y_max - y_c) / (x_max - x_c)

    def run(self, z):
        """
        Convert the input value z into a duty cycle value between duty_min
        and duty_max, if to_duty is True, otherwise convert the duty cycle into
        a value between x_min and x_max.
        """
        if self.to_duty:
            res = self.bilinear_interpolate(z, self.x_min, self.x_max,
                                            self.x_center, self.duty_min,
                                            self.duty_max, self.duty_center)
        else:
            res = self.bilinear_interpolate(z, self.duty_min, self.duty_max,
                                            self.duty_center, self.x_min,
                                            self.x_max, self.x_center)
            if self.x_deadband and abs(res - self.x_center) < self.x_deadband:
                res = self.x_center
        if self.round_digits is not None:
            res = round(res, self.round_digits)
        logger.debug(f"Input: {z} Output: {res} duty_min {self.duty_min} "
                     f"duty_max {self.duty_max} duty_center "
                     f"{self.duty_center} x_min {self.x_min} x_max "
                     f"{self.x_max} x_center {self.x_center} deadband "
                     f"{self.x_deadband}")
        return res

    def shutdown(self):
        pass


class PicoPWMInput:
    """
    Donkey part to convert a PulseIn signal from the Pico into a float output
    between out min and out max. The PulseIn signal is expected to be a list
    of integers containing hi, lo signals in microseconds.

    """
    def __init__(self, out_min=0.0, out_max=1.0, out_center=None,
                 out_deadband=None, duty_min=0.06, duty_max=0.012,
                 duty_center=None, round_digits=None):

        """
        Initialize the PicoPWMInput part.
        :param out_min:         minimum output value
        :param out_max:         maximum output value
        :param out_center:      center output value
        :param out_deadband:    if the output is within this range of the center
        :param duty_min:        minimum duty cycle
        :param duty_max:        maximum duty cycle
        :param duty_center:     center duty cycle
        """
        self.out_min = out_min
        self.out_max = out_max
        self.out_center = out_center or (out_max + out_min) / 2
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_center = duty_center or (duty_max + duty_min) / 2
        self.last_out = self.out_center
        self.last_duty = self.duty_center
        self.out_deadband = out_deadband
        self.round_digits = round_digits
        if self.round_digits is not None:
            self.last_out = round(self.last_out, self.round_digits)
        logger.info(
            f"PicoPWMInput created with min:{out_min} and max:{out_max} and "
            f"center:{self.out_center}")

    def run(self, duty_in):
        """
        Convert the duty_in value into a float output value between out_min
        and out_max.
        """
        if duty_in is not None:
            self.last_duty = duty_in
        if self.last_duty < self.duty_center:
            duty_rel = ((self.last_duty - self.duty_min)
                        / (self.duty_center - self.duty_min))
            self.last_out = (self.out_min + duty_rel
                             * (self.out_center - self.out_min))
            self.last_out = max(self.last_out, self.out_min)
        else:
            duty_rel = ((self.last_duty - self.duty_center)
                        / (self.duty_max - self.duty_center))
            self.last_out = (self.out_center + duty_rel
                             * (self.out_max - self.out_center))
            self.last_out = min(self.last_out, self.out_max)
        if (self.out_deadband and
                abs(self.last_out - self.out_center) < self.out_deadband):
            self.last_out = self.out_center
        if self.round_digits is not None:
            self.last_out = round(self.last_out, self.round_digits)
        return self.last_out, self.last_duty


class OdometerPico:
    """
    Odometric part to measure the speed usually through hall sensors sensing
    magnets attached to the drive system. Based on Pico part that delivers a
    pulse-in list of high/lo state changes.
    """
    def __init__(self, tick_per_meter=75, weight=0.5, maxlen=10, debug=False):
        """
        :param tick_per_meter: how many signals per meter
        :param weight: weighting of current measurement in average speed
                        calculation
        :param debug: if debug info should be printed
        """
        self._tick_per_meter = tick_per_meter
        self.pulses = deque(maxlen=maxlen)
        self._weight = weight
        self._max_speed = 0.0
        self._distance = 0
        self._debug_data = dict(tick=[], time=[])
        self.scale = 1.0e6 / self._tick_per_meter
        self._debug = debug
        logger.info(f"OdometerPico added with tick_per_meter: {tick_per_meter},"
                    f" weight: {weight}, maxlen: {maxlen}")

    def _weighted_avg(self):
        weighted_avg = self.pulses[0]
        for i in range(1, len(self.pulses)):
            weighted_avg = self._weight * self.pulses[i] \
                            + (1.0 - self._weight) * weighted_avg
        return weighted_avg

    def run(self, pulse_in=None):
        """
        Knowing the tick time in mu s and the ticks/m we calculate the speed. If
        ticks haven't been update since the last call we assume speed is
        zero. Then we reset the pulse history.
        :param pulse_in: list of high/lo signals in mu s
        :return speed: in m / s
        """
        if pulse_in is None:
            pulse_in = []
        self.pulses.extend(pulse_in)
        speed = 0.0
        inst_speed = 0.0
        if pulse_in:
            # for distance just count number of pulses
            self._distance += len(pulse_in)
            inst_speed = self.scale / pulse_in[-1]
            speed = self.scale / self._weighted_avg()
            self._max_speed = max(self._max_speed, speed)
            if self._debug:
                self._debug_data['time'].append(time.time())
                self._debug_data['tick'].append(pulse_in)
        else:
            self.pulses.clear()
        distance = float(self._distance) / float(self._tick_per_meter)
        logger.debug(f"Speed: {speed} InstSpeed: {inst_speed} Distance: "
                     f"{distance}")
        return speed, inst_speed, distance

    def shutdown(self):
        """
        Donkey parts interface
        """
        logger.info(f'Maximum speed {self._max_speed:4.2f}, total distance '
                    f'{self._distance / self._tick_per_meter:4.2f}')
        if self._debug:
            from os import join, getcwd
            from json import dump
            path = join(getcwd(), 'odo.json')
            with open(path, "w") as outfile:
                dump(self._debug_data, outfile, indent=4)