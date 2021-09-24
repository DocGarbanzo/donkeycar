"""
sensor.py
Classes for sensory information.

"""
from prettytable import PrettyTable
import time
from json import dump
from os.path import join
from os import getcwd
import logging

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
    def __init__(self, gpio=16, trigger=5, min_time=5.0):
        """
        :param gpio:        pin for data connection to sensor
        :param trigger:     how many consecutive readings are required for a
                            lap counter increase
        :param min_time:    how many seconds are required between laps
        """
        import pigpio
        self.gpio = gpio
        self.pi = pigpio.pi()
        self.last_time = time.time()
        self.lap_count = 0
        self.last_lap_count = 0
        self.lap_times = []
        self.lap_lenghts = []
        self.distance = 0.0
        self.last_distance = 0.0
        self.running = True
        self.count_lo = 0
        self.trigger = trigger
        self.min_time = min_time
        logger.info(f"Lap timer added at gpio {gpio}")

    def update(self):
        """
        Donkey parts interface
        """
        while self.running:
            current_state = self.pi.read(self.gpio)
            # Signal detected: if pin is lo
            if current_state == 0:
                self.count_lo += 1
                logger.debug(f'Lap timer signal low detected')
            # No signal: pin is high
            else:
                # assume when seeing enough consecutive lo this was a real
                # signal and the sensor went back to high
                if self.count_lo > self.trigger:
                    now = time.time()
                    dt = now - self.last_time
                    # only count lap if more than min_time passed
                    if dt > self.min_time:
                        self.last_time = now
                        self.lap_times.append(dt)
                        this_lap_dist = self.distance - self.last_distance
                        self.last_distance = self.distance
                        self.lap_lenghts.append(this_lap_dist)
                        logger.info(f'Lap {self.lap_count} of length '
                                    f'{this_lap_dist:6.3f}m detected after '
                                    f'{dt:6.3f}s')
                        self.lap_count += 1
                # reset lo counter
                self.count_lo = 0
            # Sleep for 0.5 ms. At 5m/s car makes 2.5mm / 0.5ms. At that speed
            # trigger determines how many cm the car has to be in the
            # absorption area of the IR signal (by default 5). This scales
            # down w/ the speed.
            time.sleep(0.0005)

    def run_threaded(self, distance):
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
        self.running = False
        logger.info("Lap Summary: (times in s)")
        pt = PrettyTable()
        pt.field_names = ['Lap', 'Time', 'Distance']
        for i, (t, l) in enumerate(zip(self.lap_times, self.lap_lenghts)):
            pt.add_row([i, f'{t:6.3f}', f'{l:6.3f}'])
        logger.info('\n' + str(pt))

    def to_list(self):
        info = [dict(lap=i, time=t, distance=l) for i, (t, l) in
                enumerate(zip(self.lap_times, self.lap_lenghts))]
        return info
