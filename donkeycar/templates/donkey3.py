#!/usr/bin/env python3
"""
Script to drive a donkey 2 car using the RC controller instead of the web
controller. Also provide a calibration of the RC throttle and
steering triggers.

Usage:
    manage.py (drive) [--pid] [--no_cam] [--model=<path_to_pilot>] [--web]\
        [--fpv] [--no_tub] [--verbose] [--type=<model_type>] [--old]
    manage.py (calibrate)
    manage.py (stream)
    manage.py (led)
    manage.py (gym) [--model=<path_to_pilot>] [--type=<model_type>] [--no_tub]\
        [--my_cfg=<path_to_myconfig.py>] [--respawn] [--random] [--verbose]
    manage.py (bench) [--model=<path_to_pilot>] [--verbose]

Options:
    -h --help               Show this screen.
    --my_cfg=myconfig.py    overwrite config file name [default: myconfig.py]
"""
import os.path

from docopt import docopt
import logging
import socket
from random import random

import donkeycar as dk
import donkeycar.parts
from donkeycar.parts.actuator import EStop
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


class Renamer:
    def run(self, data):
        return data


class LapPct:
    def __init__(self, cfg):
        self.count = 0
        self.lap = 0
        self.lap_pct = cfg.LAP_PCT
        assert isinstance(self.lap_pct, list) and len(self.lap_pct) == 3, \
            "Lap pct must be list of len 3"
        logger.info(f"Creating part LapPct: {self.lap_pct}")

    def run(self, lap, game_over=None):
        # trigger update in each new lap
        # if lap != self.lap or game_over:
        if self.count % 40 == 0:
            self.lap_pct = [-1, random(), random()]
            self.lap = lap
            logger.info(f"Setting lap pct to {self.lap_pct}")
        self.count += 1
        return self.lap_pct


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


def update_from_database(cfg, model_path, model_type):
    """ Load model database to overwrite some configs parameters with the
        values that were used in the trained model and also imply model_type
        from the trained model. """
    if not model_path:
        return model_type
    overwrite = ['TRANSFORMATIONS', 'POST_TRANSFORMATIONS', 'ROI_CROP_BOTTOM',
                 'ROI_CROP_LEFT', 'ROI_CROP_RIGHT', 'ROI_CROP_TOP',
                 'SEQUENCE_LENGTH']
    model_prefix_map = {'.tflite': 'tflite_', '.trt': 'tensorrt_',
                        '.savedmodel': '', 'h5': ''}
    db = PilotDatabase(cfg)
    model_basename, model_ext \
        = os.path.splitext(os.path.basename(model_path))
    pilot_entry = db.get_entry(model_basename)
    if pilot_entry:
        logger.info(f'Found {model_basename} in database')
        cfg_train_dict = pilot_entry['Config']
        cfg.from_dict(cfg_train_dict, overwrite)
        model_type = model_prefix_map[model_ext] + pilot_entry['Type']

    return model_type


# define some strings that are used in the vehicle data flow
CAM_IMG = 'cam/image_array'


def drive(cfg, use_pid=False, no_cam=False, model_path=None, model_type=None,
          web=False, fpv=False, no_tub=False, old=True, verbose=False):
    """
    Construct a working robotic vehicle from many parts. Each part runs as a job
    in the Vehicle loop, calling either its run or run_threaded method depending
    on the constructor flag `threaded`. All parts are updated one after another
    at the frame rate given in cfg.DRIVE_LOOP_HZ assuming each part finishes
    processing in a timely manner. Parts may have named outputs and inputs. The
    framework handles passing named outputs to parts requesting the same named
    input.
    """
    from donkeycar.parts.imu import Mpu6050Ada
    from donkeycar.parts.camera import PiCamera, FrameStreamer
    from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle, \
        RCReceiver, ModeSwitch, ThrottleOffSwitch
    from donkeycar.parts.led_status import LEDStatus

    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)
    if no_cam:
        assert model_path is None, "Can't drive with pilot but w/o camera"
    if model_path is not None:
        use_pid = True

    car = dk.vehicle.Vehicle()
    # handle record on ai: only record if cam is on and no auto-pilot ----------
    record_on_ai = getattr(cfg, 'RECORD_DURING_AI', False)
    # reduce car and camera frequency if we record on AI driving
    car_frequency = cfg.DRIVE_LOOP_HZ
    frame_rate = cfg.CAMERA_FRAMERATE
    if model_path is not None and record_on_ai \
            and hasattr(cfg, 'FREQ_REDUCTION_WITH_AI'):
        car_frequency = int(cfg.FREQ_REDUCTION_WITH_AI * car_frequency)
        frame_rate = int(cfg.FREQ_REDUCTION_WITH_AI * frame_rate)

    # add camera ---------------------------------------------------------------
    if not no_cam:
        cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                       image_d=cfg.IMAGE_DEPTH)
        car.add(cam, outputs=[CAM_IMG], threaded=True)

    # add odometer -------------------------------------------------------------
    odo = Odometer(gpio=cfg.ODOMETER_GPIO,
                   tick_per_meter=cfg.TICK_PER_M,
                   weight=0.025)
    car.add(odo, outputs=['car/speed', 'car/inst_speed', 'car/distance'])

    # add lap timer ------------------------------------------------------------
    lap = LapTimer(gpio=cfg.LAP_TIMER_GPIO, trigger=4)
    car.add(lap, inputs=['car/distance'],
            outputs=['car/lap', 'car/m_in_lap', 'car/lap_updated'],
            threaded=True)

    # add mpu ------------------------------------------------------------------
    mpu = Mpu6050Ada()
    car.add(mpu, outputs=['car/accel', 'car/gyro'], threaded=True)

    # add fpv parts ------------------------------------------------------------
    if web:
        car.add(WebFpv(), inputs=[CAM_IMG], threaded=True)
    if fpv:
        streamer = FrameStreamer(cfg.PC_HOSTNAME, cfg.FPV_PORT)
        car.add(streamer, inputs=[CAM_IMG], threaded=True)

    # create the RC receiver with 3 channels------------------------------------
    rc_steering = RCReceiver(cfg.STEERING_RC_GPIO, invert=True)
    rc_throttle = RCReceiver(cfg.THROTTLE_RC_GPIO)
    rc_wiper = RCReceiver(cfg.DATA_WIPER_RC_GPIO, jitter=0.05, no_action=0)
    car.add(rc_steering, outputs=['user/angle', 'user/angle_on'])
    car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])
    car.add(rc_wiper, outputs=['user/wiper', 'user/wiper_on'])

    # load model if present ----------------------------------------------------
    if model_path is not None:
        logger.info("Using auto-pilot")
        model_type = update_from_database(cfg, model_path, model_type)
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
            random = False
            if random:
                car.add(LapPct(cfg), inputs=['car/lap', 'game_over'],
                        outputs=['lap_pct'])
            else:
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

    # create and add the PWM steering controller
    steering_controller = PCA9685(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)

    steering_in = 'pilot/angle' if model_path else 'user/angle'
    car.add(steering, inputs=[steering_in], threaded=True)

    # create and add the PWM throttle controller for esc
    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)
    # feed signal which is either rc (user) or ai
    throttle_input = 'pid/throttle' if use_pid else 'throttle'
    car.add(EStop(car_frequency),
            inputs=[throttle_input, 'user/mode'],
            outputs=[throttle_input, 'user/estop'])
    car.add(throttle, inputs=[throttle_input], threaded=True)

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
        # old tub
        if old:
            del inputs[5], types[5]
        tub_writer = TubWriter(base_path=cfg.DATA_PATH, inputs=inputs,
                               types=types, lap_timer=lap)
        car.add(tub_writer, inputs=inputs, outputs=["tub/num_records"],
                run_condition='recording')

        # add a tub wiper that is triggered by channel 3 on the RC, but only
        # if we don't use channel 3 for switching between ai & manual
        if model_path is None:
            tub_wiper = TubWiper(tub_writer.tub, num_records=car_frequency)
            car.add(tub_wiper, inputs=['user/wiper_on'],
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
    led = LEDStatus()
    car.add(led, inputs=['user/mode', 'car/lap_updated',
                         'user/wiper_triggered'], threaded=True)
    car.add(IsThrottledChecker(), outputs=['car/throttled'], threaded=True)
    # run the vehicle
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
    from donkeycar.parts.actuator import RCReceiver

    donkey_car = dk.vehicle.Vehicle()
    # create the RC receiver
    rc_steering = RCReceiver(cfg.STEERING_RC_GPIO, invert=True)
    rc_throttle = RCReceiver(cfg.THROTTLE_RC_GPIO)
    rc_wiper = RCReceiver(cfg.DATA_WIPER_RC_GPIO, jitter=0.05, no_action=0)
    donkey_car.add(rc_steering, outputs=['user/angle', 'user/steering_on'])
    donkey_car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])
    donkey_car.add(rc_wiper, outputs=['user/wiper', 'user/wiper_on'])

    # create plotter part for printing into the shell
    class Plotter:
        def run(self, angle, steering_on, throttle, throttle_on, wiper, wiper_on):
            print('angle=%+5.4f, steering_on=%1d, throttle=%+5.4f, '
                  'throttle_on=%1d wiper=%+5.4f, wiper_on=%1d' %
                  (angle, steering_on, throttle, throttle_on, wiper, wiper_on))

    # add plotter part
    donkey_car.add(Plotter(), inputs=['user/angle', 'user/steering_on',
                                      'user/throttle', 'user/throttle_on',
                                      'user/wiper', 'user/wiper_on'])
    # run the vehicle at 10Hz to keep network traffic down
    donkey_car.start(rate_hz=10, max_loop_count=cfg.MAX_LOOPS)


def stream(cfg):
    from donkeycar.parts.camera import PiCamera, FrameStreamer
    car = dk.vehicle.Vehicle()
    hz = 20
    cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                   image_d=cfg.IMAGE_DEPTH)
    car.add(cam, outputs=['cam/image_array'], threaded=True)
    streamer = FrameStreamer(cfg.PC_HOSTNAME, cfg.FPV_PORT)
    car.add(streamer, inputs=['cam/image_array'], threaded=True)
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


def gym(cfg, model_path=None, model_type=None, no_tub=False,
        respawn=False, random=False, verbose=False):
    """
    Running donkey gym
    """
    from donkeycar.parts.dgym import GymOdometer, DonkeyGymEnv, GymLapTimer

    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)
    else:
        donkeycar.logger.setLevel(logging.INFO)

    cfg.GYM_CONF['log_level'] = donkeycar.logger.level
    car = dk.vehicle.Vehicle()
    # check if sim is running
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    is_sim = sock.connect_ex((cfg.SIM_HOST, 9091)) == 0
    sock.close()
    respawn = respawn or model_path is not None
    # respawn car at game over when driving with auto pilot
    cam = DonkeyGymEnv(cfg.DONKEY_SIM_PATH, host=cfg.SIM_HOST,
                       env_name=cfg.DONKEY_GYM_ENV_NAME, conf=cfg.GYM_CONF,
                       record_location=True,
                       record_gyroaccel=cfg.SIM_RECORD_GYROACCEL,
                       record_velocity=cfg.SIM_RECORD_VELOCITY,
                       record_lidar=cfg.SIM_RECORD_LIDAR,
                       delay=cfg.SIM_ARTIFICIAL_LATENCY,
                       new_sim=not is_sim,
                       respawn_on_game_over=respawn)
    threaded = True
    inputs = ['angle', 'throttle']
    outputs = [CAM_IMG, 'pos/pos', 'car/speed', 'pos/cte']
    if cfg.SIM_RECORD_GYROACCEL:
        outputs += ['car/gyro', 'car/accel']
    if cfg.SIM_RECORD_VELOCITY:
        outputs += ['car/vel']
    if cfg.SIM_RECORD_LIDAR:
        outputs += ['lidar/dist_array']
    outputs += ['last_lap_time', 'lap_count', 'game_over']

    car.add(cam, inputs=inputs, outputs=outputs, threaded=threaded)
    car.add(GymOdometer(), inputs=['pos/pos'], outputs=['car/distance'])
    lap_timer = GymLapTimer()
    car.add(lap_timer, inputs=['lap_count', 'last_lap_time', 'car/distance'],
            outputs=['car/lap'])

    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode=cfg.WEB_INIT_MODE)
    car.add(ctr,
            inputs=[CAM_IMG, 'tub/num_records'],
            outputs=['user/angle', 'user/throttle', 'user/mode', 'recording',
                     'buttons', 'sliders'],
            threaded=True)

# load model if present ----------------------------------------------------
    if model_path is not None:
        logger.info("Using auto-pilot")
        if not model_type:
            model_type = 'tflite_linear'

        model_type = update_from_database(cfg, model_path, model_type)
        kl = dk.utils.get_model_by_type(model_type, cfg)
        kl.load(model_path)
        kl_inputs = [CAM_IMG]
        # Add image transformations like crop or trapezoidal mask
        if hasattr(cfg, 'TRANSFORMATIONS') and cfg.TRANSFORMATIONS:
            outputs = ['cam/image_array_aug']
            car.add(ImageTransformations(cfg, 'TRANSFORMATIONS'),
                    inputs=kl_inputs, outputs=outputs)
            kl_inputs = outputs
        # imu transformation and addition AI input -----------------------------
        use_imu = 'imu' in model_path
        if use_imu:
            logger.info('Using IMU in pilot')
            imu_prep = ImuCombinerNormaliser(cfg)
            car.add(imu_prep, inputs=['car/accel', 'car/gyro'],
                    outputs=['car/imu'])
            kl_inputs.append('car/imu')

        if kl.use_lap_pct():
            if random:
                car.add(LapPct(cfg), inputs=['car/lap', 'game_over'],
                        outputs=['lap_pct'])
            else:
                car.add(SliderSorter(cfg), inputs=['sliders'],
                        outputs=['lap_pct'])

            kl_inputs.append('lap_pct')

        # add auto pilot and model reloader ------------------------------------
        kl_outputs = ['pilot/angle', 'pilot/throttle']
        model_resetter = ModelResetter(kl)
        car.add(model_resetter, inputs=['game_over'])
        car.add(kl, inputs=kl_inputs, outputs=kl_outputs)
    else:
        # rename the usr throttle
        car.add(Renamer(), inputs=['user/throttle'], outputs=['throttle'])

    # Choose what inputs should change the car.
    class DriveMode:
        def run(self, mode, user_angle, user_throttle, pilot_angle,
                pilot_throttle):
            if mode == 'user':
                return user_angle, user_throttle
            elif mode == 'local_angle':
                return pilot_angle or 0.0, user_throttle
            else:
                return pilot_angle or 0.0, pilot_throttle or 0.0
                       #pilot_throttle * cfg.AI_THROTTLE_MULT \

    car.add(DriveMode(),
            inputs=['user/mode', 'user/angle', 'user/throttle',
                    'pilot/angle', 'pilot/throttle'],
            outputs=['angle', 'throttle'])

    # if we want to record a tub -----------------------------------------------
    record_on_ai = getattr(cfg, 'RECORD_DURING_AI', False)
    if (model_path is None or record_on_ai) and not no_tub:
        inputs = [CAM_IMG, 'user/angle', 'user/throttle', 'user/mode',
                  'car/lap', 'car/distance', 'pos/pos', 'car/speed', 'pos/cte',
                  'user/state']
        types = ['image_array', 'float', 'float', 'str', 'int', 'float',
                 'vector', 'float', 'float', 'vector']
        if cfg.SIM_RECORD_GYROACCEL:
            inputs += ['car/gyro', 'car/accel']
            types += ['vector', 'vector']
        if cfg.SIM_RECORD_VELOCITY:
            inputs += ['car/vel']
            types += ['vector']
        if cfg.SIM_RECORD_LIDAR:
            inputs += ['lidar/dist_array']
            types += ['nparray']
        if record_on_ai:
            # rename ai output as user, so we can use in training
            car.add(Renamer(), inputs=['pilot/angle'], outputs=['user/angle'])
            car.add(Renamer(), inputs=['pilot/throttle'],
                    outputs=['user/throttle'])
        tub_writer = TubWriter(base_path=cfg.DATA_PATH, inputs=inputs,
                               types=types, lap_timer=lap_timer)
        # no run-condition here as we need to record backwards and zero throttle
        car.add(tub_writer, inputs=inputs, outputs=["tub/num_records"])
    # run the vehicle
    car_frequency = cfg.DRIVE_LOOP_HZ
    assert car_frequency == 20, f"Sim needs 20Hz, not {car_frequency}"
    car.start(rate_hz=car_frequency, max_loop_count=cfg.MAX_LOOPS)


def benchmark(cfg, model_path, verbose=False):
    from donkeycar.parts.camera import PiCamera
    if verbose:
        donkeycar.logger.setLevel(logging.DEBUG)
    else:
        donkeycar.logger.setLevel(logging.INFO)
    car = dk.vehicle.Vehicle()
    car_frequency = cfg.DRIVE_LOOP_HZ
    cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                   image_d=cfg.IMAGE_DEPTH)
    car.add(cam, outputs=[CAM_IMG], threaded=True)
    odo = Odometer(gpio=cfg.ODOMETER_GPIO,
                   tick_per_meter=cfg.TICK_PER_M,
                   weight=0.025)
    car.add(odo, outputs=['car/speed', 'car/inst_speed', 'car/distance'])
    lap = LapTimer(gpio=cfg.LAP_TIMER_GPIO, trigger=3)
    car.add(lap, inputs=['car/distance'],
            outputs=['car/lap', 'car/m_in_lap', 'car/lap_updated'],
            threaded=True)

        # Add image transformations like crop or trapezoidal mask
    if hasattr(cfg, 'TRANSFORMATIONS') and cfg.TRANSFORMATIONS or \
            hasattr(cfg, 'POST_TRANSFORMATIONS') and cfg.POST_TRANSFORMATIONS:
        car.add(ImageTransformations(cfg, 'TRANSFORMATIONS',
                                     'POST_TRANSFORMATIONS'),
                inputs=[CAM_IMG], outputs=[CAM_IMG])

    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT,
                                mode=cfg.WEB_INIT_MODE)
    car.add(ctr,
            inputs=[CAM_IMG, 'tub/num_records'],
            outputs=['ctr/user/angle', 'ctr/user/throttle',
                        'ctr/user/mode',
                        'ctr/recording', 'ctr/buttons', 'ctr/sliders'],
            threaded=True)

    if model_path:
        model_type = update_from_database(cfg, model_path, "")
        kl = dk.utils.get_model_by_type(model_type, cfg)
        kl.load(model_path)
        kl_inputs = [CAM_IMG]
 
        if kl.use_lap_pct():
            car.add(SliderSorter(cfg), inputs=['ctr/sliders'], outputs=['lap_pct'])
            kl_inputs.append('lap_pct')
        # add auto pilot and model reloader ------------------------------------
        kl_outputs = ['pilot/angle', 'pilot/throttle']
        car.add(kl, inputs=kl_inputs, outputs=kl_outputs)

    car.add(IsThrottledChecker(), outputs=['car/throttled'], threaded=True)
    # run the vehicle
    car.start(rate_hz=car_frequency, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    logger.info(f'Starting run of {__file__}')
    args = docopt(__doc__)
    my_cfg = args.get('--my_cfg', '')
    config = dk.load_config(myconfig=my_cfg)
    if args['drive']:
        drive(cfg=config,
              use_pid=args['--pid'],
              no_cam=args['--no_cam'],
              model_path=args['--model'],
              web=args['--web'],
              fpv=args['--fpv'],
              no_tub=args['--no_tub'],
              verbose=args['--verbose'],
              model_type=args['--type'],
              old=args['--old'])
    elif args['calibrate']:
        calibrate(config)
    elif args['stream']:
        stream(config)
    elif args['led']:
        led(config)
    elif args['gym']:
        gym(cfg=config,
            model_path=args['--model'],
            model_type=args['--type'],
            no_tub=args['--no_tub'],
            respawn=args['--respawn'],
            random=args['--random'],
            verbose=args['--verbose'])
    elif args['bench']:
        benchmark(cfg=config, model_path=args['--model'],
                  verbose=args['--verbose'])
    logger.info(f'Ending run of {__file__}')
