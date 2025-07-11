#!/usr/bin/env python
"""
Script to drive a donkey 2 car using the RC controller instead of the web
controller. Also provide a calibration of the RC throttle and
steering triggers.

Usage:
    prog drive [--pid] [--no_cam] [--model=PATH_TO_PILOT] [--web]\
        [--fpv] [--no_tub] [--verbose] [--type=MODEL_TYPE]
    prog calibrate [--verbose]
    prog stream
    prog led [--verbose]
    prog pwm [--verbose]

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

import os
from docopt import docopt
import logging
import logging.config

import donkeycar as dk
import donkeycar.parts
from donkeycar.parts.actuator import EStop, RCReceiver, PulseController, PWMSteering, \
    PWMThrottle, ModeSwitch, ThrottleOffSwitch
from donkeycar.parts.led_status import LEDStatusPi
from donkeycar.parts.pico import OdometerPico
from donkeycar.parts.pins import pwm_pin_by_id, output_pin_by_id
from donkeycar.parts.sensor import IsThrottledChecker, LapTimer
from donkeycar.parts.controller import WebFpv
from donkeycar.parts.tub_v2 import TubWiper, TubWriter
from donkeycar.pipeline.database import update_config_from_database
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.transform import ChangeDetector, ControlSwitch, ImuCombinerNormaliser, RecordingCondition, \
    SimplePidController, SpeedRescaler
from donkeycar.parts.image_transformations import ImageTransformations

from donkeycar.parts.imu import Mpu6050Ada
from donkeycar.parts.keras_2 import ModelLoader
from donkeycar.parts.web_controller.web import LocalWebController 


# Check for logging configuration file in current working directory
logging_config_path = os.path.join(os.getcwd(), 'logging.conf')
if os.path.exists(logging_config_path):
    logging.config.fileConfig(logging_config_path)

file_handler = logging.handlers.RotatingFileHandler(
    filename='./logs/log.txt', mode='a',
    maxBytes=1000000, backupCount=10)
file_handler.doRollover()

logging.basicConfig(handlers=[file_handler, logging.StreamHandler()],
                    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
                    force=True)

logger = logging.getLogger(__name__)

class Renamer:
    def run(self, data):
        return data

class SliderSorter:
    def __init__(self, cfg):
        self.lap_pct = cfg.LAP_PCT

    def run(self, sliders):
        if sliders:
            d = dict(sorted(sliders.items()))
            new_lap_pct = list(d.values())
            if new_lap_pct != self.lap_pct:
                logger.info(f'Updating lap_pct {new_lap_pct}')
            self.lap_pct = new_lap_pct
        return self.lap_pct

# define some strings that are used in the vehicle data flow
CAM_IMG = 'cam/image_array'


def drive(cfg, use_pid=False, no_cam=True, model_path=None, model_type=None,
          web=False, fpv=False, no_tub=False, verbose=False):
    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)

    car = dk.vehicle.Vehicle()
    car.mem['mode'] = 0
    car_frequency = cfg.DRIVE_LOOP_HZ
    # handle record on ai: only record if cam and auto-pilot is on
    record_on_ai = getattr(cfg, 'RECORD_DURING_AI', False)

    # add camera ---------------------------------------------------------------
    if not no_cam:
        from donkeycar.parts.camera import PiCamera
        cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                       image_d=cfg.IMAGE_DEPTH)
        car.add(cam, outputs=[CAM_IMG], threaded=True)

    rc_steering = RCReceiver(gpio=cfg.STEERING_RC_GPIO, name='steering')
    car.add(rc_steering, outputs=['user/angle', 'user/angle_on'])

    rc_throttle = RCReceiver(gpio=cfg.THROTTLE_RC_GPIO, name='throttle',)
    car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])

    rc_ch_3 = RCReceiver(min_out=0, gpio=cfg.CH3_RC_GPIO, name='ch3')
    car.add(rc_ch_3, outputs=['user/wiper', 'user/wiper_on'])

    odo = OdometerPico(tick_per_meter=cfg.TICK_PER_M, weight=0.5)
    car.add(odo, inputs=['pico/read_odo'],
            outputs=['car/speed', 'car/inst_speed', 'car/distance'])
    #
    # add lap timer ------------------------------------------------------------
    lap = LapTimer(gpio=cfg.LAP_TIMER_GPIO)
    car.add(lap, inputs=['car/distance'],
            outputs=['car/lap', 'car/m_in_lap', 'car/lap_updated'])
    #
    # # add mpu ------------------------------------------------------------------
    # mpu = Mpu6050Ada()
    # car.add(mpu, outputs=['car/accel', 'car/gyro'], threaded=True)

    # add fpv parts ------------------------------------------------------------
    if web:
        car.add(WebFpv(), inputs=[CAM_IMG], threaded=True)

    # load model if present ----------------------------------------------------
    if model_path is not None:
        logger.info("Using auto-pilot")
        model_type = update_config_from_database(cfg, model_path, model_type)
        kl = dk.utils.get_model_by_type(model_type, cfg)
        kl.load(model_path)
        kl_inputs = [CAM_IMG]
        # Add image transformations like crop or trapezoidal mask
        if hasattr(cfg, 'TRANSFORMATIONS') and cfg.TRANSFORMATIONS or \
                hasattr(cfg, 'POST_TRANSFORMATIONS') and cfg.POST_TRANSFORMATIONS:
            car.add(ImageTransformations(cfg, 'TRANSFORMATIONS',
                                         'POST_TRANSFORMATIONS'),
                    inputs=[CAM_IMG], outputs=[CAM_IMG])
        # imu transformation and addition AI input -----------------------------
        use_imu = 'imu' in model_path
        if use_imu:
            logger.info('Using IMU in pilot')
            imu_prep = ImuCombinerNormaliser(cfg)
            car.add(imu_prep, inputs=['car/accel', 'car/gyro'],
                    outputs=['car/imu'])
            kl_inputs.append('car/imu')
        elif kl.use_lap_pct():
            ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT,
                                        mode=cfg.WEB_INIT_MODE)
            car.add(ctr,
                    inputs=[CAM_IMG, 'tub/num_records'],
                    outputs=['ctr/user/angle', 'ctr/user/throttle',
                                'ctr/user/mode',
                                'ctr/recording', 'ctr/buttons', 'ctr/sliders'],
                    threaded=True)
            car.add(SliderSorter(cfg), inputs=['ctr/sliders'],
                    outputs=['lap_pct'])
            kl_inputs.append('lap_pct')
        # add auto pilot and model reloader ------------------------------------
        kl_outputs = ['pilot/angle', 'pilot/throttle']
        car.add(kl, inputs=kl_inputs, outputs=kl_outputs)
        # add file watcher and model loader so model can be reloaded
        f = FileWatcher(model_path)
        car.add(f, outputs=['model/update'])
        ml = ModelLoader(kl, model_path=model_path)
        car.add(ml, inputs=['model/update'], outputs=['model/loaded'],
                threaded=True)

        # if driving w/ ai switch between user throttle or pilot throttle by
        # pressing channel 3 on the remote control we have 2 modes,
        # pilot/steering + user/speed, or pilot/steering + pilot/speed
        mode_switch = ModeSwitch(num_modes=2, min_loops=2)
        car.add(mode_switch, inputs=['user/wiper_on'], outputs=['user/mode'])

        # This part dispatches between user or ai depending on the switch state
        switch = ControlSwitch(cfg)
        car.add(switch, inputs=['user/mode', 'user/throttle',
                                'pilot/throttle', 'pilot/angle'],
                outputs=['pilot/angle', 'throttle'])
    else:
        # rename the usr throttle
        car.add(Renamer(), inputs=['user/throttle'], outputs=['throttle'])
    if use_pid:
        # drive by pid: first convert throttle to speed
        car.add(SpeedRescaler(cfg), inputs=['throttle'], outputs=['speed'])
        # add pid controller to convert (user/pilot) speed into throttle
        pid = SimplePidController(p=cfg.PID_P, i=cfg.PID_I, d=cfg.PID_D)
        car.add(pid, inputs=['speed', 'car/inst_speed', 'user/estop'],
                outputs=['pid/throttle'])

    steering_pin = pwm_pin_by_id(cfg.STEERING_CHANNEL)
    steering_pulse = PulseController(pwm_pin=steering_pin)
    pwm_steering = PWMSteering(controller=steering_pulse,
                               left_pulse=cfg.STEERING_LEFT_PWM,
                               right_pulse=cfg.STEERING_RIGHT_PWM)
    steering_in = 'pilot/angle' if model_path else 'user/angle'
    car.add(pwm_steering, inputs=[steering_in], threaded=True)

    throttle_pin = pwm_pin_by_id(cfg.THROTTLE_CHANNEL)
    throttle_pulse = PulseController(pwm_pin=throttle_pin)
    pwm_throttle = PWMThrottle(controller=throttle_pulse,
                               max_pulse=cfg.THROTTLE_FORWARD_PWM,
                               zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                               min_pulse=cfg.THROTTLE_REVERSE_PWM)
    # feed signal which is either rc (user) or ai
    throttle_input = 'pid/throttle' if use_pid else 'throttle'
    car.add(pwm_throttle, inputs=[throttle_input], threaded=True)

    car.add(EStop(car_frequency),
            inputs=[throttle_input, 'user/mode'],
            outputs=[throttle_input, 'user/estop'])
    
    # if we want to record a tub -----------------------------------------------
    if not no_cam and (model_path is None or record_on_ai) and not no_tub:
        static_condition = None if model_path is None else record_on_ai
        rec_cond = RecordingCondition(static_condition=static_condition)
        rec_inputs = ['user/throttle_on', 'user/throttle'] \
            if model_path is None else ['dummy', 'pilot_or_user/speed']
        car.add(rec_cond, inputs=rec_inputs, outputs=['recording'])

        # add tub to save data
        inputs = [CAM_IMG, 'user/angle', 'user/throttle', 'pilot/angle',
                  'pilot/throttle', 'user/wiper_on', 'user/mode',
                  'car/speed', 'car/inst_speed', 'car/distance',
                  'car/m_in_lap', 'car/lap', 'car/accel', 'car/gyro']
        types = ['image_array', 'float', 'float', 'float',
                 'float', 'bool', 'int',
                 'float', 'float', 'float',
                 'float', 'int', 'vector', 'vector']
        # for backward compatibility remove user/wiper_on which was not in
        tub_writer = TubWriter(base_path=cfg.DATA_PATH, inputs=inputs,
                               types=types, lap_timer=lap)
        car.add(tub_writer, inputs=inputs, outputs=["tub/num_records"],
                run_condition='recording')

        # add a tub wiper that is triggered by channel 3 on the RC, but only
        # if we don't use channel 3 for switching between ai & manual
        if model_path is None:
            chgange_detector = ChangeDetector()
            car.add(chgange_detector, inputs=['user/wiper_on'],
                    outputs=['user/wiper_changed'])
            tub_wiper = TubWiper(tub_writer.tub, min_loops=1, num_records=car_frequency)
            car.add(tub_wiper, inputs=['user/wiper_changed'],
                    outputs=['user/wiper_triggered'])
        elif record_on_ai:
            class Combiner:
                def run(self, signal, mode):
                    """ Clean at switching back from auto pilot b/c there was a
                    mistake """
                    return signal and mode == 0
            car.add(Combiner(), inputs=['user/wiper_on', 'user/mode'],
                    outputs=['user/clean'])
            # delete last two seconds when we go from pilot speed to manual
            tub_wiper = TubWiper(tub_writer.tub, num_records=2 * car_frequency)
            car.add(tub_wiper, inputs=['user/clean'])

    # pressing full break for 1s will stop the car (even when wifi disconnects)
    kill_switch = ThrottleOffSwitch(min_loops=car_frequency)
    car.add(kill_switch, inputs=["user/throttle"], outputs=['user/stop'])

    car.add(LEDStatusPi(), inputs=['mode', 'car/lap_updated', 'wipe'],
            threaded=True)
    car.add(IsThrottledChecker(), outputs=['car/throttled'], threaded=True)

    car.start(rate_hz=car_frequency, max_loop_count=cfg.MAX_LOOPS)


class DigitalOutput:
    def __init__(self, gpio):
        self.pin = output_pin_by_id(gpio)
        self.pin.start()

    def run(self, value):
        self.pin.output(value > 0.0)

    def shutdown(self):
        self.pin.stop()


def pwm(cfg, verbose=False):
    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)

    car = dk.vehicle.Vehicle()
    car_frequency = cfg.DRIVE_LOOP_HZ

    rc_steering = RCReceiver(gpio=cfg.THROTTLE_RC_GPIO)
    car.add(rc_steering, outputs=['user/throttle', 'user/throttle_on'])

    # led_pin = pwm_pin_by_id('PICO.BCM.2', frequency_hz=500)
    # led_pulse = PulseController(pwm_pin=led_pin)
    # pwm_led = PWMSteering(controller=led_pulse, left_pulse=0, right_pulse=4095)
    # car.add(pwm_led, inputs=['user/throttle'])

    car.add(DigitalOutput(gpio='PICO.BCM.2'), inputs=['user/throttle'])
    car.start(rate_hz=car_frequency, max_loop_count=cfg.MAX_LOOPS)


def calibrate(cfg, verbose=False):
    """
    Construct an auxiliary robotic vehicle from only the RC controllers and
    prints their values. The RC remote usually has a tuning pot for the throttle
    and steering channel. In this loop we run the controllers and simply print
    their values in order to allow centering the RC pwm signals. If there is a
    third channel on the remote we can use it for wiping bad data while
    recording, so we print its values here, too.
    """
    class Plotter:
        def run(self, steer, throttle=0, ch_3=0):
            print(f'Calibration - angle: {steer:+4.3f} '
                  f'throttle {throttle:+4.3f} '
                  f'ch3: {ch_3:+4.3f}')

    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)

    car = dk.vehicle.Vehicle()
    rc_steering = RCReceiver(gpio=cfg.STEERING_RC_GPIO, name='steering')
    car.add(rc_steering, outputs=['user/angle', 'user/angle_on'])

    rc_throttle = RCReceiver(gpio=cfg.THROTTLE_RC_GPIO, name='throttle')
    car.add(rc_throttle, outputs=['user/throttle', 'user/rc_throttle_on'])

    rc_ch_3 = RCReceiver(min_out=0, no_action=0, gpio=cfg.CH3_RC_GPIO, name='ch3')
    car.add(rc_ch_3, outputs=['user/ch_3', 'user/rc_ch_3_on'])

    car.add(Plotter(), inputs=['user/angle', 'user/throttle', 'user/ch_3'])

    car.start(rate_hz=30, max_loop_count=cfg.MAX_LOOPS)


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


def led(cfg, verbose=False):

    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)
    car = dk.vehicle.Vehicle()
    car.add(OnOff(), outputs=['mode', 'lap', 'wipe'])
    car.add(LEDStatusPi(), inputs=['mode', 'lap', 'wipe'], threaded=True)
    car.start(rate_hz=40, max_loop_count=4000)


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
        led(config, verbose=args['--verbose'])
    elif args['pwm']:
        pwm(config, verbose=args['--verbose'])
    logger.info(f'Ending run of {__file__}')
