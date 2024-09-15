"""
sensor.py
Classes for sensory information.

"""
from prettytable import PrettyTable
import time
from json import dump
from os.path import join
from os import getcwd
from subprocess import run

import logging

from donkeycar.parts.pins import analog_input_pin_by_id

logger = logging.getLogger(__name__)


class Odometer:
    """
    Odometric part to measure the speed usually through hall sensors sensing
    magnets attached to the drive system.
    """
    def __init__(self, gpio=6, tick_per_meter=225, weight=0.5, debug=False):
        """
        :param gpio: gpio of sensor being connected
        :param tick_per_meter: how many signals per meter
        :param weight: weighting of current measurement in average speed
                        calculation
        :param debug: if debug info should be printed
        """
        import pigpio
        self._gpio = gpio
        self._tick_per_meter = tick_per_meter
        self._pi = pigpio.pi()
        self._last_tick = None
        self._last_tick_speed = None
        # as this is a time diff in mu s, make it small so it doesn't give a
        # too short average time in the first record
        self.last_tick_diff = 10000.0
        self._weight = weight
        # as this is a time diff in mu s, make it small so it doesn't give a
        # too short average time in the first record
        self._avg = 10000.0
        self.inst = 0.0
        self._max_speed = 0.0
        self._distance = 0
        self._debug = debug
        self._debug_data = dict(tick=[])
        self.scale = 1.0e6 / self._tick_per_meter

        # pigpio callback mechanics
        self._pi.set_pull_up_down(self._gpio, pigpio.PUD_UP)
        self._cb = self._pi.callback(self._gpio, pigpio.EITHER_EDGE, self._cbf)
        logger.info(f"Odometer added at gpio {gpio}")

    def _cbf(self, gpio, level, tick):
        """ Callback function for pigpio interrupt gpio. Signature is determined
        by pigpiod library. This function is called every time the gpio changes
        state as we specified EITHER_EDGE.
        :param gpio: gpio to listen for state changes
        :param level: rising/falling edge
        :param tick: # of mu s since boot, 32 bit int
        """
        import pigpio
        if self._last_tick is not None:
            diff = pigpio.tickDiff(self._last_tick, tick)
            self.inst = 0.5 * (diff + self.last_tick_diff)
            self._avg += self._weight * (self.inst - self._avg)
            self._distance += 1
            if self._debug:
                self._debug_data['tick'].append(diff)
            self.last_tick_diff = diff
        self._last_tick = tick

    def run(self):
        """
        Knowing the tick time in mu s and the ticks/m we calculate the speed. If
        ticks haven't been update since the last call we assume speed is zero
        :return speed: in m / s
        """
        speed = 0.0
        inst_speed = 0.0
        if self._last_tick_speed != self._last_tick and self.inst != 0.0:
            speed = self.scale / self._avg
            inst_speed = self.scale / self.inst
            self._max_speed = max(self._max_speed, speed)
        self._last_tick_speed = self._last_tick
        distance = float(self._distance) / float(self._tick_per_meter)
        # logger.debug(f"Speed: {speed} InstSpeed: {inst_speed} Distance: "
        #              f"{distance}")
        return speed, inst_speed, distance

    def shutdown(self):
        """
        Donkey parts interface
        """
        import pigpio
        self._cb.cancel()
        logger.info(f'Maximum speed {self._max_speed:4.2f}, total distance '
                    f'{self._distance / float(self._tick_per_meter):4.2f}')
        if self._debug:
            logger.info(f'Total num ticks {self._distance}')
            path = join(getcwd(), 'odo.json')
            with open(path, "w") as outfile:
                dump(self._debug_data, outfile, indent=4)


class LapTimer:
    """
    LapTimer to count the number of laps, and lap times, based on gpio counts
    """
    def __init__(self, gpio=16, min_time=4.0):
        """
        :param gpio:        pin for data connection to sensor
        :param trigger:     how many consecutive readings are required for a
                            lap counter increase
        :param min_time:    how many seconds are required between laps
        """
        from gpiozero import Button
        gpio = 'GPIO' + str(gpio)
        self.pin = Button(gpio, pull_up=True, hold_time=0.01, bounce_time=0.1)
        self.pin.when_held = self.count_lap
        self.last_time = time.time()
        self.lap_count = 0
        self.last_lap_count = 0
        self.lap_times = []
        self.lap_lengths = []
        self.distance = 0.0
        self.last_distance = 0.0
        self.min_time = min_time
        logger.info(f"Lap timer added at gpio {gpio}")

    def count_lap(self):
        """
        Donkey parts interface
        """
        logger.info('Lap timer triggered')
        now = time.time()
        dt = now - self.last_time
        # only count lap if more than min_time passed
        if dt > self.min_time:
            self.last_time = now
            self.lap_times.append(dt)
            this_lap_dist = self.distance - self.last_distance
            self.last_distance = self.distance
            self.lap_lengths.append(this_lap_dist)
            logger.info(f'Lap {self.lap_count} of length '
                        f'{this_lap_dist:6.3f}m detected after '
                        f'{dt:6.3f}s')
            self.lap_count += 1

    def run(self, distance):
        """
        Donkey parts interface
        """
        self.distance = distance
        lap_changed = self.lap_count != self.last_lap_count
        self.last_lap_count = self.lap_count
        return self.lap_count, distance - self.last_distance, lap_changed

    def shutdown(self):
        """
        Donkey parts interface
        """
        self.pin.pin.close()
        logger.info("Lap Summary: (times in s)")
        pt = PrettyTable()
        pt.field_names = ['Lap', 'Time', 'Distance']
        for i, (t, l) in enumerate(zip(self.lap_times, self.lap_lengths)):
            pt.add_row([i, f'{t:6.3f}', f'{l:6.3f}'])
        logger.info('\n' + str(pt))

    def to_list(self):
        info = [dict(lap=i, time=t, distance=l) for i, (t, l) in
                enumerate(zip(self.lap_times, self.lap_lengths))]
        return info


class IsThrottledChecker:
    def __init__(self):
        self.out = '0' * 19
        self.run = True

    def update(self):
        while self.run:
            cmd = "vcgencmd get_throttled"
            data = run(cmd, capture_output=True, shell=True)
            output = data.stdout.splitlines()
            # errors = data.stderr.splitlines()
            ret = output[0].decode('utf-8')
            # split off 'throttled=0x' from the hex number and convert to binary
            val = ret.split("throttled=")[1]
            logger.debug(f"Is throttled: {val}")
            out = bin(int(val[2:], 16))[2:]
            # fill to 19 digits
            len = 19
            self.out = out.zfill(len)
            if self.out[len-1] == '1':
                logger.error('Under-voltage detected')
            if self.out[len-2] == '1':
                logger.warning('Arm frequency capped')
            if self.out[len-3] == '1':
                logger.warning('Currently throttled')
            time.sleep(1.0)

    def run_threaded(self):
        return self.out

    def shutdown(self):
        self.run = False


class Voltmeter:
    """
    This is a class is a donkey part that reads the voltage from an analog
    pin using the pin factory in pins.py and reports the voltage in volts. We
    assume the analog input pin reads voltage as a 16 bit integer (eg this
    happens on Pi Pico), so we need to convert to absolut voltage. In additon
    there is a voltage divider in the circuit as the pin can only read up to
    3.3V. The voltage divider ratio is passed in the constructor.
    """

    def __init__(self, pin: str, divider_ratio: float = 4.08, warning_level=0.):
        """
        :param pin:                 the pin to read from
        :param divider_ratio:   the conversion factor from your voltage
                                    divider as the pico can only read 3.3V
                                    maximum.
        """
        # Here we initialise the pin using the analog_input_pin_by_id function
        # from the pins.py file. This is a factory function that returns
        # a pin object based on the pin name string. The pin object is
        # an AnalogInputPin object that has a read method that returns the
        # value of the pin.

        self.pin = analog_input_pin_by_id(pin)
        self.pin.start()
        self.divider_ratio = divider_ratio
        self.warning_level = warning_level
        logger.info(f"PicoVoltmeter added with pin {pin} and divider ratio "
                    f"{divider_ratio}")

    def run(self):
        """
        Reads the analog input pin value as 16 bit integer and converts it to
        a voltage using the conversion factor.

        :param pico: the pico object
        :return: the voltage in volts
        """
        value = self.pin.input()
        voltage = value * self.divider_ratio * 3.3 / 65535
        # calculate relative battery level, assuming 3s battery has > 9V and
        # minimum level should be at least 3.3V per cell
        pct = 0.0
        # 3S
        if voltage > 9.:
            pct = max(0., (voltage - 9.9) / (12.6 - 9.9))
        # 2S
        elif voltage > 6.:
            pct = max(0., (voltage - 6.6) / (8.4 - 6.6))
        else:
            logger.warning(f"Voltage below 6V: {voltage}")
        if pct < self.warning_level:
            logger.warning(f"Battery level at {int(pct * 100)}%")
        return voltage, pct

    def shutdown(self):
        pass
