"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

from abc import ABC, abstractmethod
import time
import logging
from typing import Tuple

import donkeycar as dk
from donkeycar import utils
from donkeycar.utils import clamp

logger = logging.getLogger(__name__)

try:
    import RPi.GPIO as GPIO
except ImportError as e:
    logger.warning(f"RPi.GPIO was not imported. {e}")
    globals()["GPIO"] = None

from donkeycar.parts.pins import OutputPin, PwmPin, PinState, \
    input_pwm_pin_by_id
from donkeycar.utilities.deprecated import deprecated


#
# pwm/duty-cycle/pulse
# - Standard RC servo pulses range from 1 millisecond (full reverse)
#   to 2 milliseconds (full forward) with 1.5 milliseconds being neutral (stopped).
# - These pulses are typically send at 50 hertz (every 20 milliseconds).
# - This means that, using the standard 50hz frequency, a 1 ms pulse
#   represents a 5% duty cycle and a 2 ms pulse represents a 10% duty cycle.
# - The important part is the length of the pulse;
#   it must be in the range of 1 ms to 2ms.
# - So this means that if a different frequency is used, then the duty cycle
#   must be adjusted in order to get the 1ms to 2ms pulse.
# - For instance, if a 60hz frequency is used, then a 1 ms pulse requires
#   a duty cycle of 0.05 * 60 / 50 = 0.06 (6%) duty cycle
# - We default the frequency of our PCA9685 to 60 hz, so pulses in
#   config are generally based on 60hz frequency and 12 bit values.
#   We use 12 bit values because the PCA9685 has 12 bit resolution.
#   So a 1 ms pulse is 0.06 * 4096 ~= 246, a neutral pulse of 0.09 duty cycle
#   is 0.09 * 4096 ~= 367 and full forward pulse of 0.12 duty cycles
#   is 0.12 * 4096 ~= 492
# - These are generalizations that are useful for understanding the underlying
#   api call arguments.  The final choice of duty-cycle/pulse length depends
#   on your hardware and perhaps your strategy (you may not want to go too fast,
#   and so you may choose is low max throttle pwm)
#

def duty_cycle(pulse_ms:float, frequency_hz:float) -> float:
    """
    Calculate the duty cycle, 0 to 1, of a pulse given
    the frequency and the pulse length

    :param pulse_ms:float the desired pulse length in milliseconds
    :param frequency_hz:float the pwm frequency in hertz
    :return:float duty cycle in range 0 to 1
    """
    ms_per_cycle = 1000 / frequency_hz
    duty = pulse_ms / ms_per_cycle
    return duty


def pulse_ms(pulse_bits:int) -> float:
    """
    Calculate pulse width in milliseconds given a
    12bit pulse (as a PCA9685 would use).
    Donkeycar throttle and steering PWM values are
    based on PCA9685 12bit pulse values, where
    0 is zero duty cycle and 4095 is 100% duty cycle.

    :param pulse_bits:int 12bit integer in range 0 to 4095
    :return:float pulse length in milliseconds
    """
    if pulse_bits < 0 or pulse_bits > 4095:
        raise ValueError("pulse_bits must be in range 0 to 4095 (12bit integer)")
    return pulse_bits / 4095


class PulseController:
    """
    Controller that provides a servo PWM pulse using the given PwmPin
    See pins.py for pin provider implementations.
    """

    def __init__(self, pwm_pin: PwmPin, pwm_scale: float = 1.0,
                 pwm_inverted: bool = False) -> None:
        """
        :param pwm_pin:PwnPin pin that will emit the pulse.
        :param pwm_scale:float scaling the 12 bit pulse value to compensate
                        for non-standard pwm frequencies.
        :param pwm_inverted:bool True to invert the duty cycle
        """
        self.pwm_pin = pwm_pin
        self.scale = pwm_scale
        self.inverted = pwm_inverted
        self.pwm_pin.start()

    def set_pulse(self, pulse: int) -> None:
        """
        Set the length of the pulse using a 12 bit integer (0..4095)
        :param pulse:int 12bit integer (0..4095)
        """
        if pulse < 0 or pulse > 4095:
            logger.error("pulse must be in range 0 to 4095")
            pulse = clamp(pulse, 0, 4095)

        if self.inverted:
            pulse = 4095 - pulse
        logger.debug(f"Setting pulse to {pulse}")
        self.pwm_pin.duty_cycle(int(pulse * self.scale) / 4095)

    def run(self, pulse: int) -> None:
        """
        Set the length of the pulse using a 12 bit integer (0..4095)
        :param pulse:int 12bit integer (0..4095)
        """
        self.set_pulse(pulse)

    def shutdown(self) -> None:
        self.pwm_pin.stop()


@deprecated("Deprecated in favor or PulseController.  This will be removed in a future release")
class PCA9685:
    '''
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    '''
    def __init__(self, channel, address=0x40, frequency=60,
                 busnum=None, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

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

    def set_high(self):
        self.pwm.set_pwm(self.channel, 4096, 0)

    def set_low(self):
        self.pwm.set_pwm(self.channel, 0, 4096)

    def set_duty_cycle(self, duty_cycle):
        if duty_cycle < 0 or duty_cycle > 1:
            logger.error("duty_cycle must be in range 0 to 1")
            duty_cycle = clamp(duty_cycle, 0, 1)

        if duty_cycle == 1:
            self.set_high()
        elif duty_cycle == 0:
            self.set_low()
        else:
            # duty cycle is fraction of the 12 bits
            pulse = int(4096 * duty_cycle)
            try:
                self.pwm.set_pwm(self.channel, 0, pulse)
            except OSError as e:
                logger.error(f'Problem w/ PCA9685 on channel {self.channel}: '
                             f'{e}')

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except OSError as e:
            logger.error(f'Problem w/ PCA9685 on channel {self.channel}: {e}')

    def run(self, pulse):
        self.set_pulse(pulse)


class VESC:
    '''
    VESC Motor controler using pyvesc
    This is used for most electric scateboards.

    inputs: serial_port---- port used communicate with vesc. for linux should be something like /dev/ttyACM1
    has_sensor=False------- default value from pyvesc
    start_heartbeat=True----default value from pyvesc (I believe this sets up a heartbeat and kills speed if lost)
    baudrate=115200--------- baudrate used for communication with VESC
    timeout=0.05-------------time it will try before giving up on establishing connection

    percent=.2--------------max percentage of the dutycycle that the motor will be set to
    outputs: none

    uses the pyvesc library to open communication with the VESC and sets the servo to the angle (0-1) and the duty_cycle(speed of the car) to the throttle (mapped so that percentage will be max/min speed)

    Note that this depends on pyvesc, but using pip install pyvesc will create a pyvesc file that
    can only set the speed, but not set the servo angle.

    Instead please use:
    pip install git+https://github.com/LiamBindle/PyVESC.git@master
    to install the pyvesc library
    '''
    def __init__(self, serial_port, percent=.2, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05, steering_scale = 1.0, steering_offset = 0.0 ):

        try:
            import pyvesc
        except Exception as err:
            print("\n\n\n\n", err, "\n")
            print("please use the following command to import pyvesc so that you can also set")
            print("the servo position:")
            print("pip install git+https://github.com/LiamBindle/PyVESC.git@master")
            print("\n\n\n")
            time.sleep(1)
            raise

        assert percent <= 1 and percent >= -1,'\n\nOnly percentages are allowed for MAX_VESC_SPEED (we recommend a value of about .2) (negative values flip direction of motor)'
        self.steering_scale = steering_scale
        self.steering_offset = steering_offset
        self.percent = percent

        try:
            self.v = pyvesc.VESC(serial_port, has_sensor, start_heartbeat, baudrate, timeout)
        except Exception as err:
            print("\n\n\n\n", err)
            print("\n\nto fix permission denied errors, try running the following command:")
            print("sudo chmod a+rw {}".format(serial_port), "\n\n\n\n")
            time.sleep(1)
            raise

    def run(self, angle, throttle):
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)
        self.v.set_duty_cycle(throttle*self.percent)


@deprecated("Deprecated in favor or PulseController.  This will be removed in a future release")
class PiGPIO_PWM():
    '''
    # Use the pigpio python module and daemon to get hardware pwm controls from
    # a raspberrypi gpio pins and no additional hardware. Can serve as a replacement
    # for PCA9685.
    #
    # Install and setup:
    # sudo apt update && sudo apt install pigpio python3-pigpio
    # sudo systemctl start pigpiod
    #
    # Note: the range of pulses will differ from those required for PCA9685
    # and can range from 12K to 170K
    #
    # If you use a control circuit that inverts the steering signal, set inverted to True
    # Default multipler for pulses from config etc is 100
    '''

    def __init__(self, pin, pgio=None, freq=75, inverted=False):
        import pigpio
        self.pin = pin
        self.pgio = pgio or pigpio.pi()
        self.freq = freq
        self.inverted = inverted
        self.pgio.set_mode(self.pin, pigpio.OUTPUT)
        self.dead_zone = 37000

    def __del__(self):
        self.pgio.stop()

    def set_pulse(self, pulse):
        self.output = pulse * 200
        if self.output > 0:
            self.pgio.hardware_PWM(self.pin, self.freq,
                                   int(self.output if self.inverted is False
                                       else 1e6 - self.output))

    def run(self, pulse):
        self.set_pulse(pulse)


class PWMSteering:
    """
    Wrapper over a PWM pulse controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller, left_pulse=290, right_pulse=490):

        if controller is None:
            raise ValueError("PWMSteering requires a set_pulse controller to be passed")
        set_pulse = getattr(controller, "set_pulse", None)
        if set_pulse is None or not callable(set_pulse):
            raise ValueError("controller must have a set_pulse method")
        if not utils.is_number_type(left_pulse):
            raise ValueError("left_pulse must be a number")
        if not utils.is_number_type(right_pulse):
            raise ValueError("right_pulse must be a number")

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.pulse = dk.utils.map_range(0, self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)
        self.running = True
        logger.info('PWM Steering created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, angle):
        # map absolute angle to angle that vehicle can implement.
        angle = utils.clamp(angle, self.LEFT_ANGLE, self.RIGHT_ANGLE)
        self.pulse = dk.utils.map_range(angle,
                                        self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)

    def run(self, angle):
        self.run_threaded(angle)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # set steering straight
        self.pulse = 0
        self.running = False
        self.controller.shutdown()


class PWMThrottle:
    """
    Wrapper over a PWM pulse controller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self, controller,
                 max_pulse=300, min_pulse=490, zero_pulse=350):

        if controller is None:
            raise ValueError("PWMThrottle requires a set_pulse controller to be passed")
        set_pulse = getattr(controller, "set_pulse", None)
        if set_pulse is None or not callable(set_pulse):
            raise ValueError("controller must have a set_pulse method")

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.pulse = zero_pulse

        # send zero pulse to calibrate ESC
        logger.info("Init ESC")
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        self.running = True
        logger.info('PWM Throttle created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, throttle):
        throttle = utils.clamp(throttle, self.MIN_THROTTLE, self.MAX_THROTTLE)
        if throttle > 0:
            self.pulse = dk.utils.map_range(throttle, 0, self.MAX_THROTTLE,
                                            self.zero_pulse, self.max_pulse)
        else:
            self.pulse = dk.utils.map_range(throttle, self.MIN_THROTTLE, 0,
                                            self.min_pulse, self.zero_pulse)

    def run(self, throttle):
        self.run_threaded(throttle)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)
        self.running = False
        self.controller.shutdown()


class EStop:
    """ Runs full brake for 2s if it is triggered once """
    def __init__(self, car_freq: int = 40, brake: float = -0.75):
        self.car_freq = car_freq
        self.brake = brake
        self.count = 0
        self.is_triggerd = False
        self.last_user_mode = 0

    def run(self, in_throttle: float, user_mode: int = 0):
        # E-stop gets triggered when user mode is shifted from 1 to 0
        trigger = user_mode == 0 and self.last_user_mode == 1
        self.last_user_mode = user_mode
        # if triggered and not activated, activate the brake
        if trigger and self.count == 0:
            self.count = 1
            logger.warning('E-Stop hit!')
        # if not activated return input throttle
        if self.count == 0:
            return in_throttle, trigger
        # this only runs if brake activated (i.e. self.count > 0)
        if self.count < self.car_freq:
            self.count += 1
            return self.brake, trigger
        else:
            logger.info('E-Stop released')
            self.count = 0
            return in_throttle, trigger


class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass


class L298N_HBridge_3pin(object):
    """
    Motor controlled with an L298N hbridge,
    chosen with configuration DRIVETRAIN_TYPE=DC_TWO_WHEEL_L298N
    Uses two OutputPins to select direction and
    a PwmPin to control the power to the motor.
    See pins.py for pin provider implementations.

    See https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/
    for a discussion of how the L298N hbridge module is wired in 3-pin mode.
    This also applies to the some other driver chips that emulate
    the L298N, such as the TB6612FNG motor driver.
    """

    def __init__(self, pin_forward:OutputPin, pin_backward:OutputPin, pwm_pin:PwmPin, zero_throttle:float=0, max_duty=0.9):
        """
        :param pin_forward:OutputPin when HIGH the motor will turn clockwise
                        using the output of the pwm_pin as a duty_cycle
        :param pin_backward:OutputPin when HIGH the motor will turn counter-clockwise
                            using the output of the pwm_pin as a duty_cycle
        :param pwm_pin:PwmPin takes a duty cycle in the range of 0 to 1,
                    where 0 is fully off and 1 is fully on.
        :param zero_throttle: values at or below zero_throttle are treated as zero.
        :param max_duty: the maximum duty cycle that will be send to the motors

        NOTE: if pin_forward and pin_backward are both LOW, then the motor is
            'detached' and will glide to a stop.
            if pin_forward and pin_backward are both HIGH, then the motor
            will be forcibly stopped (can be used for braking)
        """
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.pwm_pin = pwm_pin
        self.zero_throttle = zero_throttle
        self.throttle = 0
        self.max_duty = max_duty
        self.pin_forward.start(PinState.LOW)
        self.pin_backward.start(PinState.LOW)
        self.pwm_pin.start(0)

    def run(self, throttle:float) -> None:
        """
        Update the speed of the motor
        :param throttle:float throttle value in range -1 to 1,
                        where 1 is full forward and -1 is full backwards.
        """
        if throttle is None:
            logger.warn("TwoWheelSteeringThrottle throttle is None")
            return
        if throttle > 1 or throttle < -1:
            logger.warn( f"TwoWheelSteeringThrottle throttle is {throttle}, but it must be between 1(forward) and -1(reverse)")
            throttle = clamp(throttle, -1, 1)

        self.speed = throttle
        self.throttle = dk.utils.map_range_float(throttle, -1, 1, -self.max_duty, self.max_duty)
        if self.throttle > self.zero_throttle:
            self.pwm_pin.duty_cycle(self.throttle)
            self.pin_backward.output(PinState.LOW)
            self.pin_forward.output(PinState.HIGH)
        elif self.throttle < -self.zero_throttle:
            self.pwm_pin.duty_cycle(-self.throttle)
            self.pin_forward.output(PinState.LOW)
            self.pin_backward.output(PinState.HIGH)
        else:
            self.pwm_pin.duty_cycle(0)
            self.pin_forward.output(PinState.LOW)
            self.pin_backward.output(PinState.LOW)

    def shutdown(self):
        self.pwm_pin.stop()
        self.pin_forward.stop()
        self.pin_backward.stop()


class TwoWheelSteeringThrottle(object):
    """
    Modify individual differential drive wheel throttles
    in order to implemeht steering.
    """

    def run(self, throttle:float, steering:float) -> Tuple[float, float]:
        """
        :param throttle:float throttle value in range -1 to 1,
                        where 1 is full forward and -1 is full backwards.
        :param steering:float steering value in range -1 to 1,
                        where -1 is full left and 1 is full right.
        :return: tuple of left motor and right motor throttle values in range -1 to 1
                 where 1 is full forward and -1 is full backwards.
        """
        if throttle is None:
            logger.warning("TwoWheelSteeringThrottle throttle is None")
            return 0.0, 0.0
        if steering is None:
            logger.warning("TwoWheelSteeringThrottle steering is None")
            return 0.0, 0.0
        if throttle > 1 or throttle < -1:
            logger.warning( f"TwoWheelSteeringThrottle throttle is {throttle}, "
                          f"but it must be between 1(forward) and -1(reverse)")
            throttle = clamp(throttle, -1, 1)
        if steering > 1 or steering < -1:
            logger.warning( f"TwoWheelSteeringThrottle steering is {steering}, "
                          f"but it must be between 1(right) and -1(left)")
            steering = clamp(steering, -1, 1)

        left_motor_speed = throttle
        right_motor_speed = throttle

        if steering < 0:
            left_motor_speed *= (1.0 - (-steering))
        elif steering > 0:
            right_motor_speed *= (1.0 - steering)

        return left_motor_speed, right_motor_speed

    def shutdown(self) -> None:
        pass


class RCReceiver:
    """
    Class to read PWM from an RC control and convert into a float output number.
    Uses pigpio library. The code is essentially a copy of
    http://abyz.me.uk/rpi/pigpio/code/read_PWM_py.zip. You will need a voltage
    divider from a 5V RC receiver to a 3.3V Pi input pin if the receiver runs
    on 5V. If your receiver accepts 3.3V input, then it can be connected
    directly to the Pi.
    """

    def __init__(self, gpio, min_out=-1, max_out=1, min_duty=0.06,
                 max_duty=0.12, invert=False, jitter=0.025, no_action=None,
                 name=""):
        """
        :param gpio: gpio pin connected to RC channel
        :param invert: invert value of run() within [MIN_OUT,MAX_OUT]
        :param jitter: threshold below which no signal is reported
        :param no_action: value within [MIN_OUT,MAX_OUT] if no RC signal is
                          sent. This is usually zero for throttle and steering
                          being the center values when the controls are not
                          pressed.
        :param min_out: minimum output value, default -1
        :param max_out: maximum output value, default 1
        :param min_duty: minimum duty cycle, default 0.06 (6% duty cycle)
        :param max_duty: maximum duty cycle, default 0.12 (12% duty cycle)
        :param name: name of the receiver, used for logging
        
        """
        self.min_out = min_out
        self.max_out = max_out
        self.min_duty = min_duty
        self.max_duty = max_duty
        self.invert = invert
        self.jitter = jitter
        if no_action is not None:
            self.no_action = no_action
        else:
            self.no_action = (self.max_out - self.min_out) / 2.0

        self.factor = ((self.max_out - self.min_out)
                       / (self.max_duty - self.min_duty))
        self.pin = input_pwm_pin_by_id(gpio)
        self.pin.start()
        self.name = name
        logger.info(f'RCReceiver {self.name} on gpio {gpio} created')

    def run(self):
        """
        Donkey parts interface, returns pulse mapped into [MIN_OUT,MAX_OUT] or
        [MAX_OUT,MIN_OUT]
        """
        # signal is a value in [0, (MAX_OUT-MIN_OUT)]
        signal = (self.pin.duty_cycle() - self.min_duty) * self.factor
        # Assuming non-activity if the pulse is at no_action point
        is_action = abs(signal - self.no_action) > self.jitter
        # if deemed noise assume no signal
        if not is_action:
            signal = self.no_action
        # convert into min max interval
        if self.invert:
            signal = -signal + self.max_out
        else:
            signal += self.min_out
        signal = clamp(signal, self.min_out, self.max_out)
        logger.debug(f'RCReceiver {self.name} run: signal={signal}, is_action={is_action}')
        return signal, is_action

    def shutdown(self):
        """
        Donkey parts interface
        """
        self.pin.stop()


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


class L298N_HBridge_2pin(object):
    """
    Motor controlled with an 'mini' L298N hbridge using 2 PwmPins,
    one for forward pwm and for reverse pwm.
    Chosen with configuration DRIVETRAIN_TYPE=DC_TWO_WHEEL
    See pins.py for pin provider implementations.

    See https://www.instructables.com/Tutorial-for-Dual-Channel-DC-Motor-Driver-Board-PW/
    for how an L298N mini-hbridge modules is wired.
    This driver can also be used for an L9110S/HG7881 motor driver.  See
    https://electropeak.com/learn/interfacing-l9110s-dual-channel-h-bridge-motor-driver-module-with-arduino/
    for how an L9110S motor driver module is wired.
    """

    def __init__(self, pin_forward:PwmPin, pin_backward:PwmPin, zero_throttle:float=0, max_duty = 0.9):
        """
        pin_forward:PwmPin Takes a duty cycle in the range of 0 to 1,
                        where 0 is fully off and 1 is fully on.
                        When the duty_cycle > 0 the motor will turn clockwise
                        proportial to the duty_cycle
        pin_backward:PwmPin Takes a duty cycle in the range of 0 to 1,
                            where 0 is fully off and 1 is fully on.
                            When the duty_cycle > 0 the motor will turn counter-clockwise
                            proportial to the duty_cycle
        zero_throttle: values at or below zero_throttle are treated as zero.
        max_duty: the maximum duty cycle that will be send to the motors

        NOTE: if pin_forward and pin_backward are both at duty_cycle == 0,
            then the motor is 'detached' and will glide to a stop.
            if pin_forward and pin_backward are both at duty_cycle == 1,
            then the motor will be forcibly stopped (can be used for braking)
        max_duty is from 0 to 1 (fully off to fully on). I've read 0.9 is a good max.
        """
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.zero_throttle = zero_throttle
        self.max_duty = max_duty

        self.throttle=0
        self.speed=0

        self.pin_forward.start(0)
        self.pin_backward.start(0)

    def run(self, throttle:float) -> None:
        """
        Update the speed of the motor
        :param throttle:float throttle value in range -1 to 1,
                        where 1 is full forward and -1 is full backwards.
        """
        if throttle is None:
            logger.warn("TwoWheelSteeringThrottle throttle is None")
            return
        if throttle > 1 or throttle < -1:
            logger.warn( f"TwoWheelSteeringThrottle throttle is {throttle}, but it must be between 1(forward) and -1(reverse)")
            throttle = clamp(throttle, -1, 1)

        self.speed = throttle
        self.throttle = dk.utils.map_range_float(throttle, -1, 1, -self.max_duty, self.max_duty)

        if self.throttle > self.zero_throttle:
            self.pin_backward.duty_cycle(0)
            self.pin_forward.duty_cycle(self.throttle)
        elif self.throttle < -self.zero_throttle:
            self.pin_forward.duty_cycle(0)
            self.pin_backward.duty_cycle(-self.throttle)
        else:
            self.pin_forward.duty_cycle(0)
            self.pin_backward.duty_cycle(0)

    def shutdown(self):
        self.pin_forward.stop()
        self.pin_backward.stop()


class ModeSwitch:
    """
    Donkey part which allows to cycle through a number of states, every time an
    input signal is received. This is useful if for example we want to cycle
    through different behaviours in the drive mode when we only have an RC or
    similar controller with a single button but no web controller. When
    pressing that button we can cycle through different states, like drive w/
    autopilot or w/o, etc.
    As run gets called in the vehicle loop the mode switch runs only once for
    each continuous activation. A new mode switch requires to release of the
    input trigger.
    """
    def __init__(self, num_modes=1, min_loops=2):
        """
        :param int num_modes: number of modes
        :param int min_loops: min number of active loops before state is changed
        """
        assert num_modes >= 1, "Need >=1 modes in ModeSwitch part"
        self._num_modes = num_modes
        self._current_mode = 0
        self._active_loop_count = 0
        self._min_loops = min_loops
        assert min_loops > 0, "ModeSwitch equires min loops > 0"
        logger.info(f'ModeSwitch of {num_modes} modes created')

    def run(self, is_active):
        """
        Method in the vehicle loop. Cycle to next mode
        :param bool is_active: if deletion has been triggered by the caller
        :return: active mode
        """
        # only run if input is true and debounced
        if is_active:
            # increase the active loop count
            self._active_loop_count += 1
            if self._active_loop_count == self._min_loops:
                # increase internal mode by one
                self._current_mode += 1
                # if we run over the end set back to mode 0
                self._current_mode %= self._num_modes
                logger.info(f"Switched to mode {self._current_mode}")
        else:
            # trigger released, reset active loop count
            self._active_loop_count = 0

        return self._current_mode


class ThrottleOffSwitch(ModeSwitch):
    """ Use ModeSwitch when pressing negative throttle for a bit to stop the
        car."""
    def __init__(self, min_loops=40, throttle_val=-0.6):
        super().__init__(num_modes=2, min_loops=min_loops)
        self._throttle_val = throttle_val

    def run(self, throttle_in):
        trigger = throttle_in < self._throttle_val
        stop = super().run(trigger) == 1
        if stop:
            logger.info("Stopping the car by negative throttle")
        return stop


