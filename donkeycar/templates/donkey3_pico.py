#!/usr/bin/env python
"""
Script to drive a donkey 2 car using the RC controller instead of the web
controller. Also provide a calibration of the RC throttle and
steering triggers.

Usage:
    prog drive [--pid] [--no_cam] [--model=PATH_TO_PILOT] [--web]\
        [--fpv] [--no_tub] [--verbose] [--type=MODEL_TYPE]
    prog calibrate
    prog stream
    prog led

Options:
    -h --help               Show this screen.
    --my_cfg=myconfig.py    overwrite config file name [default: myconfig.py]
    --pid                   use pid
    --no_cam                don't use camera and don't try loading camera
                            module
    --model=PATH_TO_PILOT   path to the model to load
    --web                   use web fpv
    --fpv                   use standalone fpv
    --no_tub                don't write to tub
    --verbose               set logging level to debug
    --type=MODEL_TYPE       type of the model to load [default: linear]
"""

import os.path

from docopt import docopt
import logging
import time
from datetime import datetime
import socket
from random import random

import donkeycar as dk
import donkeycar.parts
from donkeycar.parts.actuator import EStop
from donkeycar.parts.pico import OdometerPico
from donkeycar.parts.tub_v2 import TubWiper, TubWriter
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.keras_2 import ModelLoader, ModelResetter
from donkeycar.parts.transform import SimplePidController, \
    ImuCombinerNormaliser, ControlSwitch, \
    SpeedRescaler, RecordingCondition
from donkeycar.parts.sensor import Odometer, LapTimer, IsThrottledChecker
from donkeycar.parts.controller import WebFpv
from donkeycar.parts.web_controller.web import LocalWebController
from donkeycar.parts.image_transformations import ImageTransformations
from donkeycar.pipeline.database import PilotDatabase


file_handler = logging.handlers.RotatingFileHandler(
    filename='./logs/log.txt', mode='a',
    maxBytes=1000000, backupCount=10)
file_handler.doRollover()

logging.basicConfig(handlers=[file_handler, logging.StreamHandler()],
                    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
                    force=True)

logger = logging.getLogger(__name__)

# define some strings that are used in the vehicle data flow
CAM_IMG = 'cam/image_array'


def drive(cfg, use_pid=False, no_cam=True, model_path=None, model_type=None,
          web=False, fpv=False, no_tub=False, verbose=False):
    """
    Construct a working robotic vehicle from many parts. Each part runs as a job
    in the Vehicle loop, calling either its run or run_threaded method depending
    on the constructor flag `threaded`. All parts are updated one after another
    at the frame rate given in cfg.DRIVE_LOOP_HZ assuming each part finishes
    processing in a timely manner. Parts may have named outputs and inputs. The
    framework handles passing named outputs to parts requesting the same named
    input.
    """
    from donkeycar.parts.pico import Pico, DutyScaler

    class Plotter:
        def run(self, angle, steer, throttle, gas, odo, ch_3,):
            print(f'Ts: {datetime.now().isoformat()} angle: {angle:+5.4f} '
                  f'steer duty: {steer:+5.4f} throttle {throttle:+5.4f} '
                  f'throttle duty {gas:+5.4f} odo: {odo} ch3: {ch_3:+5.4f} ')

    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)

    car = dk.vehicle.Vehicle()
    car_frequency = cfg.DRIVE_LOOP_HZ

    # add camera ---------------------------------------------------------------
    if not no_cam:
        from donkeycar.parts.camera import PiCamera
        cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                       image_d=cfg.IMAGE_DEPTH)
        car.add(cam, outputs=[CAM_IMG], threaded=True)

    pico = Pico(pin_configuration=cfg.PICO_PIN_CONFIGURATION)
    car.add(pico, outputs=['pico/read_steering_duty', 'pico/read_throttle_duty',
                           'pico/read_ch_3_duty', 'pico/read_odo'],
            threaded=True)

    rc_steering = DutyScaler(x_min=-1, x_max=1,
                             duty_min=cfg.PICO_STEERING_MIN_DUTY,
                             duty_max=cfg.PICO_STEERING_MAX_DUTY,
                             duty_center=cfg.PICO_STEERING_CENTER_DUTY,
                             x_deadband=0.01,
                             to_duty=False)
    car.add(rc_steering, inputs=['pico/read_steering_duty'],
            outputs=['user/angle'])

    rc_throttle = DutyScaler(x_min=-1, x_max=1,
                             duty_min=cfg.PICO_THROTTLE_MIN_DUTY,
                             duty_max=cfg.PICO_THROTTLE_MAX_DUTY,
                             duty_center=cfg.PICO_THROTTLE_CENTER_DUTY,
                             x_deadband=0.01)
    car.add(rc_throttle, inputs=['pico/read_throttle_duty'],
            outputs=['user/throttle'])

    rc_ch_3 = DutyScaler(x_min=0, x_max=1, to_duty=False, round_digits=0)
    car.add(rc_ch_3, inputs=['pico/read_ch_3_duty'], outputs=['user/ch_3'])

    pwm_steering = DutyScaler(x_min=-1, x_max=1,
                              duty_min=cfg.PICO_STEERING_MIN_DUTY,
                              duty_max=cfg.PICO_STEERING_MAX_DUTY,
                              duty_center=cfg.PICO_STEERING_CENTER_DUTY,
                              to_duty=True)
    car.add(pwm_steering, inputs=['user/angle'],
            outputs=['pico/write_steering_duty'])

    car.add(Plotter(), inputs=['user/angle', 'rc/steering_duty',
                               'user/throttle', 'rc/throttle_duty',
                               'pico/read_odo', 'user/ch_3',])

    # add odometer -------------------------------------------------------------
    odo = OdometerPico(tick_per_meter=cfg.TICK_PER_M, weight=0.5)
    car.add(odo, inputs=['pico/read_odo'],
            outputs=['car/speed', 'car/inst_speed', 'car/distance'])
    #
    # add lap timer ------------------------------------------------------------
    lap = LapTimer(gpio=cfg.LAP_TIMER_GPIO, trigger=4)
    car.add(lap, inputs=['car/distance'],
            outputs=['car/lap', 'car/m_in_lap', 'car/lap_updated'],
            threaded=True)
    #
    # # add mpu ------------------------------------------------------------------
    # mpu = Mpu6050Ada()
    # car.add(mpu, outputs=['car/accel', 'car/gyro'], threaded=True)
    car.start(rate_hz=car_frequency, max_loop_count=cfg.MAX_LOOPS)


def calibrate(cfg):
    """
    Construct an auxiliary robotic vehicle from only the RC controllers and
    prints their values. The RC remote usually has a tuning pot for the throttle
    and steering channel. In this loop we run the controllers and simply print
    their values in order to allow centering the RC pwm signals. If there is a
    third channel on the remote we can use it for wiping bad data while
    recording, so we print its values here, too.
    """
    from donkeycar.parts.pico import Pico, PicoPWMInput, DutyScaler

    class Plotter:
        def run(self, steer, steer_duty=0, throttle=0, throttle_duty=0,
                ch_3=0, ch_3_duty=0):
            print(f'Ts: {datetime.now().isoformat()} angle: {steer} '
                  f'steer duty: {steer_duty:+4.3f} throttle {throttle:+4.3f} '
                  f'throttle duty {throttle_duty:+4.3f} '
                  f'ch3: {ch_3:+4.3f} ch3 duty: {ch_3_duty:+4.3f}')

    car = dk.vehicle.Vehicle()
    pico = Pico(pin_configuration=cfg.PICO_PIN_CONFIGURATION)
    car.add(pico, outputs=['pico/read_steering_duty', 'pico/read_throttle_duty',
                           'pico/read_ch_3_duty', 'pico/read_odo'],
            threaded=True)

    rc_steering = DutyScaler(x_min=-1, x_max=1,
                             duty_min=cfg.PICO_STEERING_MIN_DUTY,
                             duty_max=cfg.PICO_STEERING_MAX_DUTY,
                             duty_center=cfg.PICO_STEERING_CENTER_DUTY,
                             x_deadband=0.01, to_duty=False)
    car.add(rc_steering, inputs=['pico/read_steering_duty'],
            outputs=['user/angle'])

    rc_throttle = DutyScaler(x_min=-1, x_max=1,
                             duty_min=cfg.PICO_THROTTLE_MIN_DUTY,
                             duty_max=cfg.PICO_THROTTLE_MAX_DUTY,
                             duty_center=cfg.PICO_THROTTLE_CENTER_DUTY,
                             x_deadband=0.01, to_duty=False)
    car.add(rc_throttle, inputs=['pico/read_throttle_duty'],
            outputs=['user/throttle'])

    rc_ch_3 = DutyScaler(x_min=0, x_max=1,
                         duty_min=cfg.PICO_STEERING_MIN_DUTY,
                         duty_max=cfg.PICO_STEERING_MAX_DUTY, round_digits=0,
                         to_duty=False)
    car.add(rc_ch_3, inputs=['pico/read_ch_3_duty'], outputs=['user/ch_3'])

    pwm_steering = DutyScaler(x_min=-1, x_max=1,
                              duty_min=cfg.PICO_STEERING_MIN_DUTY,
                              duty_max=cfg.PICO_STEERING_MAX_DUTY,
                              duty_center=cfg.PICO_STEERING_CENTER_DUTY,
                              to_duty=True)
    car.add(pwm_steering, inputs=['user/angle'],
            outputs=['pico/write_steering_duty'])

    car.add(Plotter(), inputs=['user/angle', 'pico/read_steering_duty'])
                               # 'user/throttle', 'pico/read_throttle_duty',
                               # 'user/ch_3', 'pico/read_ch_3_duty'])

    # add odometer -------------------------------------------------------------
    # odo = OdometerPico(tick_per_meter=cfg.TICK_PER_M, weight=0.5)
    # car.add(odo, inputs=['pico/read_odo'],
    #         outputs=['car/speed', 'car/inst_speed', 'car/distance'])
    #
    # add lap timer ------------------------------------------------------------
    # lap = LapTimer(gpio=cfg.LAP_TIMER_GPIO, trigger=4)
    # car.add(lap, inputs=['car/distance'],
    #         outputs=['car/lap', 'car/m_in_lap', 'car/lap_updated'],
    #         threaded=True)
    #
    # # add mpu ------------------------------------------------------------------
    # mpu = Mpu6050Ada()
    # car.add(mpu, outputs=['car/accel', 'car/gyro'], threaded=True)
    car.start(rate_hz=10, max_loop_count=cfg.MAX_LOOPS)


def stream(cfg):
    from donkeycar.parts.camera import PiCamera, FrameStreamer
    car = dk.vehicle.Vehicle()
    hz = 20
    cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                   image_d=cfg.IMAGE_DEPTH)
    car.add(cam, outputs=['cam/image_array'], threaded=True)
    # streamer = FrameStreamer(cfg.PC_HOSTNAME, cfg.FPV_PORT)
    # car.add(streamer, inputs=['cam/image_array'], threaded=True)
    car.add(WebFpv(), inputs=['cam/image_array'], threaded=True)
    car.start(rate_hz=hz, max_loop_count=cfg.MAX_LOOPS)


class OnOff:
    count = 1
    mode = 0

    def run(self):
        if self.count % 400 == 0:
            self.mode = 1 - self.mode
        is_lap = self.count % 240 == 0
        is_wipe = self.count % 280 == 0
        self.count += 1
        return self.mode, is_lap, is_wipe


def led(cfg):
    from donkeycar.parts.led_status import LEDStatus
    donkeycar.logger.setLevel(logging.DEBUG)
    car = dk.vehicle.Vehicle()
    car.add(OnOff(), outputs=['mode', 'lap', 'wipe'])
    car.add(LEDStatus(), inputs=['mode', 'lap', 'wipe'], threaded=True)
    car.start(rate_hz=40, max_loop_count=2000)


if __name__ == '__main__':
    logger.info(f'Starting run of {__file__}')
    args = docopt(__doc__)
    # my_cfg = args.get('--my_cfg')
    # config = dk.load_config(myconfig=my_cfg)
    config = dk.load_config()
    if args['drive']:
        drive(cfg=config,
              use_pid=args['--pid'],
              no_cam=args['--no_cam'],
              model_path=args['--model'],
              web=args['--web'],
              fpv=args['--fpv'],
              no_tub=args['--no_tub'],
              verbose=args['--verbose'],
              model_type=args['--type'])
    elif args['calibrate']:
        calibrate(config)
    elif args['stream']:
        stream(config)
    elif args['led']:
        led(config)
    logger.info(f'Ending run of {__file__}')
