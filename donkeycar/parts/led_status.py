import time
from threading import Thread, active_count

import RPi.GPIO as GPIO
from donkeycar.parts.actuator import PCA9685
import logging
from collections import deque
import queue


logger = logging.getLogger(__name__)


class LED:
    ''' 
    Toggle a GPIO pin for led control
    '''
    def __init__(self, pin):
        self.pin = pin

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.blink_changed = 0
        self.on = False

    def toggle(self, condition):
        if condition:
            GPIO.output(self.pin, GPIO.HIGH)
            self.on = True
        else:
            GPIO.output(self.pin, GPIO.LOW)
            self.on = False            

    def blink(self, rate):
        if time.time() - self.blink_changed > rate:
            self.toggle(not self.on)
            self.blink_changed = time.time()

    def run(self, blink_rate):
        if blink_rate == 0:
            self.toggle(False)
        elif blink_rate > 0:
            self.blink(blink_rate)
        else:
            self.toggle(True)

    def shutdown(self):
        self.toggle(False)        
        GPIO.cleanup()


class RGB_LED:
    ''' 
    Toggle a GPIO pin on at max_duty pwm if condition is true, off if condition is false.
    Good for LED pwm modulated
    '''
    def __init__(self, pin_r, pin_g, pin_b, invert_flag=False):
        self.pin_r = pin_r
        self.pin_g = pin_g
        self.pin_b = pin_b
        self.invert = invert_flag
        print('setting up gpio in board mode')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_r, GPIO.OUT)
        GPIO.setup(self.pin_g, GPIO.OUT)
        GPIO.setup(self.pin_b, GPIO.OUT)
        freq = 50
        self.pwm_r = GPIO.PWM(self.pin_r, freq)
        self.pwm_g = GPIO.PWM(self.pin_g, freq)
        self.pwm_b = GPIO.PWM(self.pin_b, freq)
        self.pwm_r.start(0)
        self.pwm_g.start(0)
        self.pwm_b.start(0)
        self.zero = 0
        if( self.invert ):
            self.zero = 100

        self.rgb = (50, self.zero, self.zero)

        self.blink_changed = 0
        self.on = False

    def toggle(self, condition):
        if condition:
            r, g, b = self.rgb
            self.set_rgb_duty(r, g, b)
            self.on = True
        else:
            self.set_rgb_duty(self.zero, self.zero, self.zero)
            self.on = False

    def blink(self, rate):
        if time.time() - self.blink_changed > rate:
            self.toggle(not self.on)
            self.blink_changed = time.time()

    def run(self, blink_rate):
        if blink_rate == 0:
            self.toggle(False)
        elif blink_rate > 0:
            self.blink(blink_rate)
        else:
            self.toggle(True)

    def set_rgb(self, r, g, b):
        r = r if not self.invert else 100-r
        g = g if not self.invert else 100-g
        b = b if not self.invert else 100-b
        self.rgb = (r, g, b)
        self.set_rgb_duty(r, g, b)

    def set_rgb_duty(self, r, g, b):
        self.pwm_r.ChangeDutyCycle(r)
        self.pwm_g.ChangeDutyCycle(g)
        self.pwm_b.ChangeDutyCycle(b)

    def shutdown(self):
        self.toggle(False)
        GPIO.cleanup()


# Colors
class Color12bit:
    RED = (4095, 0, 0)
    GREEN = (0, 4095, 0)
    BLUE = (0, 0, 4095)
    YELLOW = (4095, 512, 0)
    PURPLE = (1024, 0, 4095)
    WHITE = (4095, 1048, 4095)
    ORANGE = (4095, 128, 0)
    OFF = (0, 0, 0)


class LEDStatus:
    def __init__(self, r_channel=13, g_channel=14, b_channel=15):
        self.rgb_pins \
            = (PCA9685(r_channel), PCA9685(g_channel), PCA9685(b_channel))
        self.run = True
        self.pulse_on_time = 0.25
        self.queue = queue.Queue()
        self.pulse_color = Color12bit.GREEN
        self.continuous_loop = True
        self.larsen(4)
        logger.info("Created LEDStatus part")

    def _set_color(self, color):
        for c, pin in zip(color, self.rgb_pins):
            pin.set_pulse(c)

    def pulse(self):
        """ Produces blinking continuous signal """
        while self.continuous_loop:
            self.blink(self.pulse_on_time, self.pulse_color, 1)
        # set back indicator to allow thread to start again
        self.continuous_loop = True

    def blink(self, delay, color, num, short=True):
        """
        Blinks num times with on-time = delay and off-time = 2 * delay
        :param delay:   How long in s
        :param color:   rgb tuple in [0, 4095]
        :param num:     How often
        :param short:   If short time on and long time off or vice versa
        :return:        None
        """
        on_time = delay
        off_time = 2 * on_time
        if not short:
            on_time, off_time = off_time, on_time
        for _ in range(num):
            self._set_color(color)
            time.sleep(on_time)
            self._set_color(Color12bit.OFF)
            time.sleep(off_time)

    def larsen(self, num):
        on_time = 0.15
        colors = (Color12bit.BLUE, Color12bit.GREEN, Color12bit.RED)
        for _ in range(num):
            for col in colors:
                self._set_color(col)
                time.sleep(on_time)
        self._set_color(Color12bit.OFF)

    def schedule_continuous(self):
        self.queue.put(Thread(target=self.pulse, daemon=True))

    def update(self):
        # start the continuous thread
        self.schedule_continuous()
        # this is the queue worker
        while self.run:
            i = self.queue.get()
            i.start()
            i.join()
            self.queue.task_done()

    def run_threaded(self, mode=None, lap=False, wipe=False):
        if mode is not None:
            self.pulse_color = Color12bit.GREEN \
                if mode == 0 else Color12bit.YELLOW
        t = None
        if lap:
            # 3 red blinks when lap
            t = Thread(target=self.blink, args=(0.1, Color12bit.RED, 3))
        if wipe:
            # 1 violet blink when wiper
            t = Thread(target=self.blink, args=(0.5, Color12bit.PURPLE, 1, False))
        # if new thread created, schedule it and restart continuous thread:
        if t:
            self.continuous_loop = False
            self.queue.put(t)
            self.schedule_continuous()

    def shutdown(self):
        # stop the loop
        self.run = False
        self.continuous_loop = False
        self.blink(0.3, Color12bit.WHITE, 2, False)


class ColorRGB:
    RED = (1, 0, 0)
    GREEN = (0, 1, 0)
    BLUE = (0, 0, 1)
    YELLOW = (1, 1, 0)
    ORANGE = (1, 0.5, 0)
    PURPLE = (0.75, 0, 1)
    WHITE = (1, 1, 1)
    OFF = (0, 0, 0)


class LEDStatusPi:
    def __init__(self, r_pin='GPIO6', g_pin='GPIO13', b_pin='GPIO19'):
        from gpiozero import RGBLED
        self.led = RGBLED(red=r_pin, g_pin=g_pin, b_pin=b_pin)
        self.run = True
        self.queue = queue.Queue()
        self.pulse_color = ColorRGB.GREEN
        self.mode = 0

    def update(self):
        while self.run:
            kwargs = dict(background=False)
            kwargs.update(self.queue.get())
            self.led.blink(**kwargs)
            self.queue.task_done()
            self.blink_continuous()

    def _update_continuous(self, mode=0):
        if mode == self.mode:
            return
        if mode == 0:
            self.pulse_color = ColorRGB.GREEN
        elif mode == 1:
            self.pulse_color = ColorRGB.YELLOW
        self.blink_continuous()
        self.mode = mode

    def run_threaded(self, mode=0, lap=False, wipe=False):
        self._update_continuous(mode)
        t = None
        # 3 red blinks when lap or 1 violet blink when wiper
        if lap:
            t = dict(on_color=ColorRGB.RED, on_time=0.2, off_time=0.2, n=3)
        if wipe:
            t = dict(on_color=ColorRGB.PURPLE, on_time=0.4, off_time=0.0, n=1)
        if t:
            self.queue.put(t)

    def shutdown(self):
        self.run = False
        self.led.blink(on_color=ColorRGB.WHITE, on_time=0.4,
                       off_time=0.2, background=False, n=2)

    def blink_continuous(self):
        self.led.blink(on_color=self.pulse_color, on_time=0.3,
                       off_time=0.3, background=True)


if __name__ == "__main__":
    import time
    import sys
    pin_r = int(sys.argv[1])
    pin_g = int(sys.argv[2])
    pin_b = int(sys.argv[3])
    rate = float(sys.argv[4])
    print('output pin', pin_r, pin_g, pin_b, 'rate', rate)

    p = RGB_LED(pin_r, pin_g, pin_b)
    
    iter = 0
    while iter < 50:
        p.run(rate)
        time.sleep(0.1)
        iter += 1
    
    delay = 0.1

    iter = 0
    while iter < 100:
        p.set_rgb(iter, 100-iter, 0)
        time.sleep(delay)
        iter += 1
    
    iter = 0
    while iter < 100:
        p.set_rgb(100 - iter, 0, iter)
        time.sleep(delay)
        iter += 1

    p.shutdown()

