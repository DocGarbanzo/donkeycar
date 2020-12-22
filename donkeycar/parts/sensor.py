"""
sensor.py
Classes for sensory information.

"""
from prettytable import PrettyTable
import time
from json import dump
from os.path import join
from os import getcwd


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
        self.last_tick_diff = 0
        self._weight = weight
        self._avg = 0.0
        self.inst = 0.0
        self._max_speed = 0.0
        self._distance = 0
        self._debug = debug
        self._debug_data = dict(tick=[])
        self.scale = 1.0e6 / self._tick_per_meter

        # pigpio callback mechanics
        self._pi.set_pull_up_down(self._gpio, pigpio.PUD_UP)
        self._cb = self._pi.callback(self._gpio, pigpio.EITHER_EDGE, self._cbf)
        print("Odometer added at gpio {}".format(gpio))

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
            self._avg = self._weight * self.inst  \
                        + (1.0 - self._weight) * self._avg
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
        if self._debug:
            print("Speed =", speed, "InstSpeed = ", inst_speed)
        return speed, inst_speed

    def shutdown(self):
        """
        Donkey parts interface
        """
        import pigpio
        self._cb.cancel()
        print('Maximum speed {0:4.2f}, total distance {1:4.2f}'
              .format(self._max_speed,
                      float(self._distance) / float(self._tick_per_meter)))
        if self._debug:
            print('Total num ticks {}'.format(self._distance))
            path = join(getcwd(), 'odo.json')
            with open(path, "w") as outfile:
                dump(self._debug_data, outfile, indent=4)


class LapTimer:
    """
    LapTimer to count the number of laps, and lap times, based on gpio counts
    """
    def __init__(self, gpio=16, trigger=5, min_time=1.0, debug=None):
        """
        :param gpio: gpio of sensor being connected
        :param debug: if debug info should be printed
        """
        import pigpio
        self.gpio = gpio
        self.pi = pigpio.pi()
        self.last_time = time.time()
        self.lap_count = 0
        self.lap_times = []
        self.debug = debug
        self.running = True
        self.count_lo = 0
        self.trigger = trigger
        self.min_time = min_time
        print("LapTimerThreaded added at gpio {}".format(gpio))

    def update(self):
        """
        Donkey parts interface
        """
        while self.running:
            current_state = self.pi.read(self.gpio)
            # Signal detected: if pin is lo
            if current_state == 0:
                self.count_lo += 1
            # No signal: pin is high
            else:
                # assume when seeing enough consecutive lo this was a real
                # signal and the sensor went back to high
                if self.count_lo > self.trigger:
                    now = time.time()
                    dt = now - self.last_time
                    # only count lap if more than min_time passed
                    if dt > self.min_time:
                        print('Lap {0} detected after {1:6.3f}s'
                              .format(self.lap_count, dt))
                        self.last_time = now
                        self.lap_count += 1
                        self.lap_times.append(dt)
                # rest lo counter
                self.count_lo = 0
            # Sleep for 0.5 ms. At 5m/s car makes 2.5mm / 0.5ms. At that speed
            # trigger determines how many cm the car has to be in the
            # absorption area of the IR signal (by default 5). This scales
            # down w/ the speed.
            time.sleep(0.0005)

    def run_threaded(self):
        """
        Donkey parts interface
        """
        return self.lap_count

    def shutdown(self):
        """
        Donkey parts interface
        """
        self.running = False
        print("Lap Summary: (times in s)")
        pt = PrettyTable()
        pt.field_names = ['Lap', 'Time']
        for i, t in enumerate(self.lap_times):
            pt.add_row([i, '{0:6.3f}'.format(t)])
        print(pt)


