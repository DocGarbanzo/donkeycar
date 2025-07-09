# -*- coding: utf-8 -*-
import logging
from simple_pid import PID
from donkeycar.utils import *


logger = logging.getLogger(__name__)


class Lambda:
    """
    Wraps a function into a donkey part.
    """
    def __init__(self, f):
        """
        Accepts the function to use.
        """
        self.f = f

    def run(self, *args, **kwargs):
        return self.f(*args, **kwargs)

    def shutdown(self):
        return


class TriggeredCallback:
    def __init__(self, args, func_cb):
        self.args = args
        self.func_cb = func_cb

    def run(self, trigger):
        if trigger:
            self.func_cb(self.args)

    def shutdown(self):
        return


class DelayedTrigger:
    def __init__(self, delay):
        self.ticks = 0
        self.delay = delay

    def run(self, trigger):
        if self.ticks > 0:
            self.ticks -= 1
            if self.ticks == 0:
                return True

        if trigger:
            self.ticks = self.delay

        return False

    def shutdown(self):
        pass


class PIDController:
    """ Performs a PID computation and returns a control value.
        This is based on the elapsed time (dt) and the current value of the
        process variable (i.e. the thing we're measuring and trying to change).
        https://github.com/chrisspen/pid_controller/blob/master/pid_controller/pid.py
    """

    def __init__(self, p=0, i=0, d=0, debug=False):

        # initialize gains
        self.Kp = p
        self.Ki = i
        self.Kd = d

        # initialize delta t variables
        self.prev_tm = time.time()
        self.prev_err = 0
        self.totalError = 0

        # debug flag (set to True for console output)
        self.debug = debug
        self.last_alpha = 0.0

    def run(self, setpoint, feedback):
        # Error and time calculation
        err = feedback - setpoint
        curr_tm = time.time()
        # Calculate time differential.
        dt = curr_tm - self.prev_tm
        # error differential
        dif_error = err - self.prev_err
        # integral error
        self.totalError += err

        # Initialize output variable.
        curr_alpha = 0.0
        # Add proportional component.
        curr_alpha += -self.Kp * err
        # Add integral component.
        curr_alpha += -self.Ki * self.totalError * dt

        # Add differential component (avoiding divide-by-zero).
        if dt > 0:
            curr_alpha += self.Kd * dif_error / float(dt)

        # Maintain memory for next loop.
        self.prev_tm = curr_tm
        self.prev_err = err

        if self.debug:
            print('PID error={0:4.3f} total_error={1:4.3f} dif_error={2:4.3f} '
                  'output={3:4.3f}'
                  .format(err, self.totalError, dif_error, curr_alpha))

        self.last_alpha = curr_alpha
        return curr_alpha

    def shutdown(self):
        pass


class SimplePidController:
    """
    Donkey part wrap of SimplePid https://github.com/m-lundberg/simple-pid
    """
    def __init__(self, p, i, d):
        self.pid = PID(Kp=p, Ki=i, Kd=d)
        self.pid.output_limits = (0, 1)
        logger.info(f"Created SimplePid part p: {p}, i:{i}, d:{d}")

    def run(self, set_point, feedback, e_stop=False):
        self.pid.setpoint = set_point
        out = self.pid(feedback)
        if e_stop:
            self.pid.reset()
            out = 0
        logger.debug(f'PID setpoint: {set_point:4.2f} feedback:'
                     f' {feedback:4.2f} e_stop: {e_stop}')
        return out


class ImgBrightnessNormaliser:
    """
    Donkey part to normalise the image brightness, works on the original
    image with 8-bit int values [0, 255]
    """
    def __init__(self, norm=100, proportional=True):
        """
        :param norm: Normalisation factor between 0 (black) and 255 (white)
        """
        self.norm = norm
        self.relative = proportional
        print('Created part', type(self).__name__, 'using',
              'proportional' if proportional else 'absolute pixel shifting')

    def run(self, img_arr):
        """
        Method to adjust the brightness level to given value using PIL
        :param img_arr:     numpy input image array of ints in [0, 255]
        :return:            numpy image array of ints in [0, 255]
        """
        # determine the average brightness
        brightness = self.brightness(img_arr)
        # this number should be between 0 and 255
        assert 0 <= brightness <= 255, \
            'brightness {:2f} is wrong'.format(brightness)
        if self.relative:
            # if brightness is zero we can't scale, so return input
            if brightness == 0:
                return img_arr
            # this is the factor to be applied
            factor = self.norm / brightness
            # convert to float first and adjust
            img_arr = img_arr.astype(np.float32)
            img_arr *= factor
            # clamp to [0, 255]
            np.clip(img_arr, 0, 255, out=img_arr)
            return img_arr.astype(np.uint8)
        else:
            img_arr_out = img_arr + int(self.norm) - brightness
            np.clip(img_arr_out, 0, 255, out=img_arr_out)
            return img_arr_out

    @staticmethod
    def brightness(img_arr):
        # determine number of pixels = h * w * num-channels
        num = 1
        for j in img_arr.shape:
            num *= j
        # determine the average brightness
        brightness = img_arr.sum() / num
        return brightness


class ImuCombinerNormaliser:
    def __init__(self, cfg):
        self.cfg = cfg
        self.accel_factor = 1.0 / cfg.IMU_ACCEL_NORM
        self.gyro_factor = 1.0 / cfg.IMU_GYRO_NORM

    def run(self, accel, gyro):
        combined = clamp_and_norm(accel, self.accel_factor) + \
                   clamp_and_norm(gyro, self.gyro_factor)
        # crop to number of imu degrees to be used in model
        if hasattr(self.cfg, 'IMU_DIM'):
            combined = combined[:self.cfg.IMU_DIM]
        return combined


class ControlSwitch:
    """ Class to switch between user speed or pilot speed """
    def __init__(self, cfg):
        self.throttle_mult = cfg.AI_THROTTLE_MULT
        self.angle_mult = getattr(cfg, 'AI_ANGLE_MULT', self.throttle_mult)
        logger.info(f"Creating ControlSwitch with throttle mult = "
                    f"{self.throttle_mult}, angle mult = {self.angle_mult}")

    def run(self, user_mode, user_speed, pilot_speed, pilot_angle):
        # currently assume 2 modes 0 = user, 1 = pilot
        if user_mode < 1:
            return pilot_angle * self.angle_mult, user_speed
        else:
            return (pilot_angle * self.angle_mult,
                    pilot_speed * self.throttle_mult)


class SteeringSwitch:
    """ Class to switch between user speed or pilot steering """
    def run(self, user_angle, pilot_angle, user_mode):
        return user_angle if user_mode == 0 else pilot_angle


class SpeedRescaler:
    """ Class to rescale normalised [0,1] speed to car speed """
    def __init__(self, cfg):
        self.max_speed = cfg.MAX_SPEED

    def run(self, norm_speed):
        return norm_speed * self.max_speed


class RecordingCondition:
    """ Class to switch recording based on static or dynamic condition """
    def __init__(self, static_condition=None):
        self.condition = static_condition

    def run(self, dynamic_condition, throttle_val):
        logger.debug(f"RecordingCondition: static={self.condition}, "
                     f"dynamic={dynamic_condition}, throttle={throttle_val}")
        if self.condition is None:
            return dynamic_condition and throttle_val > 0
        else:
            return self.condition and throttle_val > 0


class ChangeDetector:
    """ Class to detect changes in boolean input values """
    def __init__(self):
        self.previous_value = None

    def run(self, input_value):
        if self.previous_value is None:
            self.previous_value = input_value
            return False
        
        changed = input_value != self.previous_value
        self.previous_value = input_value
        return changed
