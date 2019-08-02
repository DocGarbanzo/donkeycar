"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk

        
class PCA9685:
    '''
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            # replace the get_bus function with our own

            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay) # "Tamiya TBLE-02" makes a little leap otherwise

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)


class PiGPIO_PWM():
    '''
    # Use the pigpio python module and daemon to get hardware pwm controls from
    # a raspberrypi gpio pins and no additional hardware. Can serve as a replacement
    # for PCA9685.
    #
    # Install and setup:
    # sudo update && sudo apt install pigpio python3-pigpio
    # sudo systemctl start pigpiod
    #
    # Note: the range of pulses will differ from those required for PCA9685
    # and can range from 12K to 170K
    '''

    def __init__(self, pin, pgio=None, freq=75):
        import pigpio

        self.pin = pin
        self.pgio = pgio or pigpio.pi()
        self.freq = freq
        self.pgio.set_mode(self.pin, pigpio.OUTPUT)

    def __del__(self):
        self.pgio.stop()

    def set_pulse(self, pulse):
        self.pgio.hardware_PWM(self.pin, self.freq, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)


class JHat:
    '''
    PWM motor controler using Teensy emulating PCA9685.
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        print("Firing up the Hat")
        import Adafruit_PCA9685
        LED0_OFF_L = 0x08
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            # replace the get_bus function with our own

            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        self.register = LED0_OFF_L+4*channel

        # we install our own write that is more efficient use of interrupts
        self.pwm.set_pwm = self.set_pwm

    def set_pulse(self, pulse):
        self.set_pwm(self.channel, 0, pulse)

    def set_pwm(self, channel, on, off):
        # sets a single PWM channel
        self.pwm._device.writeList(self.register, [off & 0xFF, off >> 8])

    def run(self, pulse):
        self.set_pulse(pulse)


class JHatReader:
    '''
    Read RC controls from teensy
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        import Adafruit_PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.register = 0  # i2c read doesn't take an address
        self.steering = 0
        self.throttle = 0
        self.running = True
        # send a reset
        self.pwm._device.writeRaw8(0x06)

    def read_pwm(self):
        '''
        send read requests via i2c bus to Teensy to get
        pwm control values from last RC input
        '''
        h1 = self.pwm._device.readU8(self.register)
        # first byte of header must be 100, otherwize we might be reading
        # in the wrong byte offset
        while h1 != 100:
            print("skipping to start of header")
            h1 = self.pwm._device.readU8(self.register)

        # h2 ignored now
        h2 = self.pwm._device.readU8(self.register)

        val_a = self.pwm._device.readU8(self.register)
        val_b = self.pwm._device.readU8(self.register)
        self.steering = (val_b << 8) + val_a

        val_c = self.pwm._device.readU8(self.register)
        val_d = self.pwm._device.readU8(self.register)
        self.throttle = (val_d << 8) + val_c

        # scale the values from -1 to 1
        self.steering = (((float)(self.steering)) - 1500.0) / 500.0 + 0.158
        self.throttle = (((float)(self.throttle)) - 1500.0) / 500.0 + 0.136

    def update(self):
        while(self.running):
            self.read_pwm()
            time.sleep(0.015)

    def run_threaded(self):
        return self.steering, self.throttle

    def shutdown(self):
        self.running = False
        time.sleep(0.1)


class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        print('PWM Steering created')

    def run(self, angle):
        # map absolute angle to angle that vehicle can implement.
        pulse = dk.utils.map_range(angle,
                                self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        # set steering straight
        self.run(0)


class PWMThrottle:
    """
    Wrapper over a PWM motor controller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        # send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.max_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.min_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        print('PWM Throttle created')

    def run(self, throttle):
        if throttle > 0:
            pulse = dk.utils.map_range(throttle, 0, self.MAX_THROTTLE,
                                       self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.utils.map_range(throttle, self.MIN_THROTTLE, 0,
                                       self.min_pulse, self.zero_pulse)
        self.controller.set_pulse(pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)



class Adafruit_DCMotor_Hat:
    '''
    Adafruit DC Motor Controller
    Used for each motor on a differential drive car.
    '''
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
        import atexit

        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60)

        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num

        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0

    def run(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(dk.utils.map_range(abs(speed), -1, 1, -255, 255))

        if speed > 0:
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)

        self.motor.setSpeed(self.throttle)

    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)

class Maestro:
    '''
    Pololu Maestro Servo controller
    Use the MaestroControlCenter to set the speed & acceleration values to 0!
    '''
    import threading

    maestro_device = None
    astar_device = None
    maestro_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency=60):
        import serial

        if Maestro.maestro_device is None:
            Maestro.maestro_device = serial.Serial('/dev/ttyACM0', 115200)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Maestro.astar_device == None:
            Maestro.astar_device = serial.Serial('/dev/ttyACM2', 115200, timeout= 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds
        w *= 4  # in quarter microsenconds the maestro wants
        w = int(w)

        with Maestro.maestro_lock:
            Maestro.maestro_device.write(bytearray([ 0x84,
                                                     self.channel,
                                                     (w & 0x7F),
                                                     ((w >> 7) & 0x7F)]))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def readline(self):
        ret = None
        with Maestro.astar_lock:
            # expecting lines like
            # E n nnn n
            if Maestro.astar_device.inWaiting() > 8:
                ret = Maestro.astar_device.readline()

        if ret is not None:
            ret = ret.rstrip()

        return ret


class Teensy:
    '''
    Teensy Servo controller
    '''
    import threading

    teensy_device = None
    astar_device = None
    teensy_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Teensy.teensy_device is None:
            Teensy.teensy_device = serial.Serial('/dev/teensy', 115200, timeout=0.01)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Teensy.astar_device == None:
            Teensy.astar_device = serial.Serial('/dev/astar', 115200, timeout=0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds

        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("%c %.1f\n" % (self.channel, w)).encode('ascii'))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def teensy_readline(self):
        ret = None
        with Teensy.teensy_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.teensy_device.inWaiting() > 8:
                ret = Teensy.teensy_device.readline()

        if ret is not None:
            ret = ret.rstrip()

        return ret

    def astar_readline(self):
        ret = None
        with Teensy.astar_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.astar_device.inWaiting() > 8:
                ret = Teensy.astar_device.readline()

        if ret is not None:
            ret = ret.rstrip()

        return ret


class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass


class L298N_HBridge_DC_Motor(object):
    '''
    Motor controlled with an L298N hbridge from the gpio pins on Rpi
    '''
    def __init__(self, pin_forward, pin_backward, pwm_pin, freq = 50):
        import RPi.GPIO as GPIO
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.pwm_pin = pwm_pin

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_forward, GPIO.OUT)
        GPIO.setup(self.pin_backward, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.pwm_pin, freq)
        self.pwm.start(0)

    def run(self, speed):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        max_duty = 90 #I've read 90 is a good max
        self.throttle = int(dk.utils.map_range(speed, -1, 1, -max_duty, max_duty))

        if self.throttle > 0:
            self.pwm.ChangeDutyCycle(self.throttle)
            GPIO.output(self.pin_forward, GPIO.HIGH)
            GPIO.output(self.pin_backward, GPIO.LOW)
        elif self.throttle < 0:
            self.pwm.ChangeDutyCycle(-self.throttle)
            GPIO.output(self.pin_forward, GPIO.LOW)
            GPIO.output(self.pin_backward, GPIO.HIGH)
        else:
            self.pwm.ChangeDutyCycle(self.throttle)
            GPIO.output(self.pin_forward, GPIO.LOW)
            GPIO.output(self.pin_backward, GPIO.LOW)

    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm.stop()
        GPIO.cleanup()


class TwoWheelSteeringThrottle(object):

    def run(self, throttle, steering):
        if throttle > 1 or throttle < -1:
            raise ValueError( "throttle must be between 1(forward) and -1(reverse)")

        if steering > 1 or steering < -1:
            raise ValueError( "steering must be between 1(right) and -1(left)")

        left_motor_speed = throttle
        right_motor_speed = throttle

        if steering < 0:
            left_motor_speed *= (1.0 + steering)
        elif steering > 0:
            right_motor_speed *= (1.0 - steering)

        return left_motor_speed, right_motor_speed


class RCReceiver:
    """
    Class to read PWM from an RC control and convert into a float output number.
    Uses pigpio library. The code is essentially a copy of
    http://abyz.me.uk/rpi/pigpio/code/read_PWM_py.zip. You will need a voltage
    divider from a 5V RC receiver to a 3.3V Pi input pin if the receiver runs
    on 5V. If your receiver accepts 3.3V input, then it can be connected
    directly to the Pi.
    """
    MIN_OUT = -1
    MAX_OUT = 1

    def __init__(self, gpio, invert=False, jitter=0.025, no_action=None):
        """
        :param gpio: gpio pin connected to RC channel
        :param invert: invert value of run() within [MIN_OUT,MAX_OUT]
        :param jitter: threshold below which no signal is reported
        :param no_action: value within [MIN_OUT,MAX_OUT] if no RC signal is
                            sent. This is usually zero for throttle and steering
                            being the center values when the controls are not
                            pressed.
        """
        import pigpio
        self.pi = pigpio.pi()
        self.gpio = gpio
        self._high_tick = None
        self._period = None
        self._high = None
        self._min_pwm = 1000
        self._max_pwm = 2000
        self._invert = invert
        self._jitter = jitter
        if no_action is not None:
            self._no_action = no_action
        else:
            self._no_action = (self.MAX_OUT - self.MIN_OUT) / 2.0

        self.pi.set_mode(self.gpio, pigpio.INPUT)
        self._cb = self.pi.callback(self.gpio, pigpio.EITHER_EDGE, self._cbf)
        print('RCReceiver gpio ' + str(gpio) + ' created')

    def _update_param(self, tick):
        """ Helper function for callback function _cbf.
        :param tick: current tick in mu s
        :return: difference in ticks
        """
        import pigpio
        if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)
            return t

    def _cbf(self, gpio, level, tick):
        """ Callback function for pigpio interrupt gpio. Signature is determined
            by pigpiod library. This function is called every time the gpio
            changes state as we specified EITHER_EDGE.
        :param gpio: gpio to listen for state changes
        :param level: rising/falling edge
        :param tick: # of mu s since boot, 32 bit int
        """
        if level == 1:
            self._period = self._update_param(tick)
            self._high_tick = tick
        elif level == 0:
            self._high = self._update_param(tick)

    def pulse_width(self):
        """
        :return: the PWM pulse width in microseconds.
        """
        if self._high is not None:
            return self._high
        else:
            return 0.0

    def run(self):
        """
        Donkey parts interface, returns pulse mapped into [MIN_OUT,MAX_OUT] or
        [MAX_OUT,MIN_OUT]
        """
        # signal is a value in [0, (MAX_OUT-MIN_OUT)]
        signal = (self.pulse_width() - self._min_pwm) / (self._max_pwm - self._min_pwm) \
            * (self.MAX_OUT - self.MIN_OUT)
        # Assuming non-activity if the pulse is in the middle of [MIN_OUT,MAX_OUT]
        is_action = abs(signal - self._no_action) > self._jitter
        if not self._invert:
            return signal + self.MIN_OUT, is_action
        else:
            return -signal + self.MAX_OUT, is_action

    def shutdown(self):
        """
        Donkey parts interface
        """
        import pigpio
        self._cb.cancel()


class MockRCReceiver:
    """
    Mock of the above to run on host for debugging
    """
    def __init__(self):
        self.is_run_ = False

    def run(self):
        if self.is_run_ is True:
            self.is_run_ = False
            return 0.5, True
        else:
            self.is_run_ = True
            return 0.0, False

    def shutdown(self):
        pass


class Mini_HBridge_DC_Motor_PWM(object):
    '''
    Motor controlled with an mini hbridge from the gpio pins on Rpi
    This can be using the L298N as above, but wired differently with only
    two inputs and no enable line.
    https://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Dtoys-and-games&field-keywords=Mini+Dual+DC+Motor+H-Bridge+Driver
    https://www.aliexpress.com/item/5-pc-2-DC-Motor-Drive-Module-Reversing-PWM-Speed-Dual-H-Bridge-Stepper-Motor-Mini
    '''
    def __init__(self, pin_forward, pin_backward, freq = 50, max_duty = 90):
        '''
        max_duy is from 0 to 100. I've read 90 is a good max.
        '''
        import RPi.GPIO as GPIO
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.max_duty = max_duty

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_forward, GPIO.OUT)
        GPIO.setup(self.pin_backward, GPIO.OUT)

        self.pwm_f = GPIO.PWM(self.pin_forward, freq)
        self.pwm_f.start(0)
        self.pwm_b = GPIO.PWM(self.pin_backward, freq)
        self.pwm_b.start(0)

    def run(self, speed):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed is None:
            return

        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(dk.utils.map_range(speed, -1, 1, -self.max_duty, self.max_duty))

        if self.throttle > 0:
            self.pwm_f.ChangeDutyCycle(self.throttle)
            self.pwm_b.ChangeDutyCycle(0)
        elif self.throttle < 0:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(-self.throttle)
        else:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm_f.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.pwm_f.stop()
        self.pwm_b.stop()
        GPIO.cleanup()


def map_frange(x, X_min, X_max, Y_min, Y_max):
    '''
    Linear mapping between two ranges of values, helper function for below
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range/Y_range

    y = ((x - X_min) / XY_ratio + Y_min)

    return y


class RPi_GPIO_Servo(object):
    '''
    Servo controlled from the gpio pins on Rpi
    '''
    def __init__(self, pin, freq = 50, min=5.0, max=7.8):
        import RPi.GPIO as GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.pin, freq)
        self.pwm.start(0)
        self.min = min
        self.max = max

    def run(self, pulse):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        #I've read 90 is a good max
        self.throttle = map_frange(pulse, -1.0, 1.0, self.min, self.max)
        #print(pulse, self.throttle)
        self.pwm.ChangeDutyCycle(self.throttle)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm.stop()
        GPIO.cleanup()


class ServoBlaster(object):
    '''
    Servo controlled from the gpio pins on Rpi
    This uses a user space service to generate more efficient PWM via DMA control blocks.
    Check readme and install here:
    https://github.com/richardghirst/PiBits/tree/master/ServoBlaster
    cd PiBits/ServoBlaster/user
    make
    sudo ./servod
    will start the daemon and create the needed device file:
    /dev/servoblaster

    to test this from the command line:
    echo P1-16=120 > /dev/servoblaster

    will send 1200us PWM pulse to physical pin 16 on the pi.

    If you want it to start on boot:
    sudo make install
    '''
    def __init__(self, pin):
        self.pin = pin
        self.servoblaster = open('/dev/servoblaster', 'w')
        self.min = min
        self.max = max

    def set_pulse(self, pulse):
        s = 'P1-%d=%d\n' % (self.pin, pulse)
        self.servoblaster.write(s)
        self.servoblaster.flush()

    def run(self, pulse):
        self.set_pulse(pulse)

    def shutdown(self):
        self.run((self.max + self.min) / 2)
        self.servoblaster.close()


class ModeSwitch:
    """
    Donkey part which allows to cycle through a number of states, every time an
    input signal is received. This is useful if for example we want to cycle
    through different behaviours in the drive mode when we only have an RC or
    similar controller but no web controller. When pressing that button we can
    cycle through different states, like drive w/ auto pilot or w/o, etc.
    As run gets called in the vehicle loop the mode switch runs only once for
    each continuous activation. A new mode switch requires to release of the
    input trigger.
    """
    def __init__(self, num_modes=1):
        """
        :param num_modes: number of modes
        """
        assert num_modes >= 1, "Need >=1 modes in ModeSwitch part"
        self._num_modes = num_modes
        self._current_mode = 0
        self._active_loop = False

    def run(self, is_active):
        """
        Method in the vehicle loop. Cycle to next mode
        :param is_active: if deletion has been triggered by the caller
        :return: active mode
        """
        # only run if input is true and debounced
        if is_active:
            if not self._active_loop:
                # action command
                self._current_mode += 1
                # if we run over the end set back to mode 0
                if self._current_mode == self._num_modes:
                    self._current_mode = 0
                # activate the loop tracker
                self._active_loop = True
                print("Switched to mode", self._current_mode)
        else:
            # trigger released, reset active loop
            self._active_loop = False

        return self._current_mode
