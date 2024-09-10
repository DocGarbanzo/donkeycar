import atexit
import time
from collections import deque

import serial
import json
import logging
from threading import Lock, Thread

from serial.serialutil import SerialTimeoutException

logger = logging.getLogger(__name__)


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
            'GP13': dict(mode='INPUT', pull_up=False),
            'GP18': dict(mode='PULSE_IN', maxlen=8, auto_clear=True),
            'GP16': dict(mode='PULSE_IN', maxlen=4),
            'GP28': dict(mode='ANALOG_IN'),
        },
        'output_pins': {
            'GP14': dict(mode='OUTPUT', value=0),
            'GP15': dict(mode='PWM', frequency=60, duty_cycle=0.06),
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
        Initialize the Pico communicator.
        :param port: port for data connection
        """
        self.serial = serial.Serial(port, 115200)  #, write_timeout=10.0)
        self.counter = 0
        self.running = True
        self.pin_configuration = dict()
        self.send_dict = dict()
        # need to guarantee dictionary order as we might receive the
        # dictionary with a different order of keys after deserialisation
        self.receive_dict = dict()
        self.lock = Lock()
        self.start = None
        logger.info(f"Creating Pico on port: {port}...initialising comms, ...")
        # send the initial setup dictionary to clear all pins
        pack = json.dumps(dict(input_pins={}, output_pins={})) + '\n'
        try:
            self.serial.write(pack.encode())
            self.t = Thread(target=self.loop, args=(), daemon=True)
            self.t.start()
            atexit.register(self.stop)
        except SerialTimeoutException as e:
            logger.error(f"Failed to initialise Pi Pico dict because of {e}")
            raise RuntimeError("Failed to initialise Pi Pico.")
        logger.info(f"...Pico communication initialised.")

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
                logger.error(f'Problem with serial comms {e}')
            self.counter += 1
        logger.info('Pico loop stopped.')

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
        with self.lock:
            if gpio not in self.receive_dict:
                msg = (f"Pin {gpio} not in receive_dict. Known pins: "
                       f"{', '.join(self.receive_dict.keys())}")
                logger.error(msg)
                raise RuntimeError(msg)
            return self.receive_dict[gpio]

    def stop(self):
        self.running = False
        time.sleep(0.1)
        #self.t.join()
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
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
        assert mode in ('INPUT', 'PULSE_IN', 'ANALOG_IN', 'PWM_IN'), \
            f"Mode {mode} not supported for input pins."

        setup_dict = dict(input_pins={gpio: dict(mode=mode, **kwargs)})
        logger.info(f"Setting up input pin {gpio} in mode {mode} using "
                    f"setup dict {setup_dict}")
        try:
            with self.lock:
                # send the setup dictionary
                pack = json.dumps(setup_dict) + '\n'
                logger.debug(f"Sending setup dict: {pack}")
                self.serial.write(pack.encode())
        except SerialTimeoutException as e:
            logger.error(f"Input pin {gpio} setup failed to send setup dict "
                         f"because of {e}, skipping.")

        self.receive_dict[gpio] = 0

    def setup_output_pin(self, gpio: str, mode: str, **kwargs) -> None:
        """
        :param gpio:    the gpio pin to set up
        :param mode:    the mode of the pin
        :param kwargs:  additional arguments for the mode
        """

        assert mode in ('OUTPUT', 'PWM'), \
            f"Mode {mode} not supported for output pins on Pico"
        setup_dict = dict(output_pins={gpio: dict(mode=mode, **kwargs)})
        logger.info(f"Setting up output pin {gpio} in mode {mode} using "
                    f"setup dict {setup_dict}")
        try:
            with self.lock:
                # send the setup dictionary
                pack = json.dumps(setup_dict) + '\n'
                self.serial.write(pack.encode())
                time.sleep(0.2)
                self.send_dict[gpio] = 0 if mode == 'OUTPUT' else kwargs['duty']
        except SerialTimeoutException as e:
            logger.error(f"Output pin {gpio} setup failed to send setup dict "
                         f"because of {e}, skipping.")

    def remove_pin(self, gpio: str) -> None:
        """
        :param gpio:    the gpio pin to remove
        """
        setup_dict = dict()
        logger.info(f"Removing pin {gpio}")
        if gpio in self.receive_dict:
            setup_dict['input_pins'] = {gpio: {}}
            del self.receive_dict[gpio]
        elif gpio in self.send_dict:
            setup_dict['output_pins'] = {gpio: {}}
            del self.send_dict[gpio]
        else:
            logger.warning(f"Pin {gpio} not in send or receive dict.")
            return
        try:
            with self.lock:
                # send the setup dictionary
                pack = json.dumps(setup_dict) + '\n'
                self.serial.write(pack.encode())
        except SerialTimeoutException as e:
            logger.error(f"Remove pin {gpio} setup failed to send setup dict "
                         f"because of {e}, skipping.")

instance = Pico()


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