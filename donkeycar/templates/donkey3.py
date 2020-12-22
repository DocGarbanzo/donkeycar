#!/usr/bin/env python3
"""
Script to drive a donkey 2 car using the RC controller instead of the web
controller and to do a calibration of the RC throttle and steering triggers.

Usage:
    manage.py (drive) [--pid] [--no_cam] [--model=<path_to_pilot>] [--web] \
    [--fpv] [--no_tub] [--verbose]
    manage.py (calibrate)
    manage.py (stream)

Options:
    -h --help        Show this screen.
"""

from docopt import docopt

import donkeycar as dk
from donkeycar.parts.camera import PiCamera, FrameStreamer
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle, \
    RCReceiver, ModeSwitch
from donkeycar.parts.datastore import TubWiper, TubHandler
from donkeycar.parts.clock import Timestamp
from donkeycar.parts.transform import SimplePidController, ImgPrecondition
from donkeycar.parts.sensor import Odometer, LapTimer
from donkeycar.parts.controller import WebFpv
from donkeycar.parts.imu import Mpu6050Ada


class TypePrinter:
    def __init__(self, type_name):
        self.type_name = type_name

    def run(self, in_type):
        print("Type of", self.type_name, type(in_type))


def drive(cfg, use_pid=False, no_cam=False, model_path=None,
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
    if no_cam:
        assert model_path is None, "Can't drive with pilot but w/o camera"

    car = dk.vehicle.Vehicle()

    clock = Timestamp()
    car.add(clock, outputs=['timestamp'])

    # only record if cam is on and no auto-pilot
    record_on_ai = cfg.RECORD_DURING_AI if hasattr(cfg, 'RECORD_DURING_AI') \
        else False

    car_frequency = cfg.DRIVE_LOOP_HZ
    frame_rate = cfg.CAMERA_FRAMERATE
    if model_path is not None and record_on_ai \
            and hasattr(cfg, 'FREQ_REDUCTION_WITH_AI'):
        car_frequency = int(cfg.FREQ_REDUCTION_WITH_AI * car_frequency)
        frame_rate = int(cfg.FREQ_REDUCTION_WITH_AI * frame_rate)

    if not no_cam:
        cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                       image_d=cfg.IMAGE_DEPTH, framerate=frame_rate)
        car.add(cam, outputs=['cam/image_array'], threaded=True)

    odo = Odometer(gpio=cfg.ODOMETER_GPIO,
                   tick_per_meter=cfg.TICK_PER_M,
                   weight=0.025,
                   debug=verbose)
    car.add(odo, outputs=['car/speed', 'car/inst_speed'])
    lap = LapTimer(gpio=cfg.LAP_TIMER_GPIO, trigger=4)
    car.add(lap, outputs=['car/lap'], threaded=True)
    mpu = Mpu6050Ada()
    car.add(mpu, outputs=['car/accel', 'car/gyro'], threaded=True)

    if web:
        car.add(WebFpv(), inputs=['cam/image_array'], threaded=True)

    if fpv:
        streamer = FrameStreamer(cfg.PC_HOSTNAME, cfg.FPV_PORT)
        car.add(streamer, inputs=['cam/image_array'], threaded=True)

    # create the RC receiver with 3 channels
    rc_steering = RCReceiver(cfg.STEERING_RC_GPIO, invert=True)
    rc_throttle = RCReceiver(cfg.THROTTLE_RC_GPIO)
    rc_wiper = RCReceiver(cfg.DATA_WIPER_RC_GPIO, jitter=0.05, no_action=0)
    car.add(rc_steering, outputs=['user/angle', 'user/steering_on'])
    car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])
    car.add(rc_wiper, outputs=['user/wiper', 'user/wiper_on'])

    pilot_throttle_var = 'pilot/throttle'
    # if pid we want to convert throttle to speed
    if use_pid:
        class Rescaler:
            def run(self, controller_input):
                return controller_input * cfg.MAX_SPEED

        car.add(Rescaler(), inputs=['user/throttle'], outputs=['user/speed'])
        pilot_throttle_var = 'pilot/speed'

    # load model if present
    if model_path is not None:
        print("Using auto-pilot")
        if '3d' in model_path:
            model_type = '3d'
        else:
            model_type = 'tflite_linear' if '.tflite' in model_path else 'linear'

        kl = dk.utils.get_model_by_type(model_type, cfg)
        kl.load(model_path)

        car.add(ImgPrecondition(cfg), inputs=['cam/image_array'],
                outputs=['cam/normalized/cropped'])

        use_imu = 'imu' in model_path
        inputs = ['cam/normalized/cropped']
        outputs = ['pilot/angle', pilot_throttle_var]
        if use_imu:
            print('Use IMU in pilot')

            class ImuCombiner:
                def run(self, accel, gyro):
                    combined = accel + gyro
                    # crop to number of imu degrees to be used in model
                    if hasattr(cfg, 'IMU_DIM'):
                        combined = combined[:cfg.IMU_DIM]
                    return combined

            car.add(ImuCombiner(), inputs=['car/accel', 'car/gyro'],
                    outputs=['car/imu'])
            inputs.append('car/imu')

        car.add(kl, inputs=inputs, outputs=outputs)
        # if driving w/ ai switch between user throttle or pilot throttle by
        # pressing chanel 3 on the remote control
        mode_switch = ModeSwitch(num_modes=2)
        car.add(mode_switch, inputs=['user/wiper_on'], outputs=['user/mode'])

        # This part dispatches between user or ai depending on the switch state
        class PilotCondition:
            def run(self, user_mode, user_var, pilot_var):
                if user_mode == 0:
                    return user_var
                else:
                    return pilot_var * cfg.AI_THROTTLE_MULT

        # switch between user or pilot speed (if pid) or throttle (if no pid)
        var = 'speed' if use_pid else 'throttle'
        car.add(PilotCondition(),
                inputs=['user/mode', 'user/' + var, 'pilot/' + var],
                outputs=[var])

    # use pid either for rc control output or for ai output
    speed = 'speed' if model_path is not None else 'user/speed'
    # drive by pid w/ speed
    if use_pid:
        # add pid controller to convert throttle value into speed
        pid = SimplePidController(p=cfg.PID_P, i=cfg.PID_I, d=cfg.PID_D,
                                  debug=verbose)
        car.add(pid, inputs=[speed, 'car/inst_speed'], outputs=['throttle'])

    # create and add the PWM steering controller
    steering_controller = PCA9685(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)
    # feed signal which is either rc (user) or ai
    input_field = 'user/angle' if model_path is None else 'pilot/angle'
    car.add(steering, inputs=[input_field], threaded=True)

    # create and add the PWM throttle controller for esc
    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)
    # feed signal which is either rc (user) or ai
    input_field = 'user/throttle' if not use_pid and model_path is None \
        else 'throttle'
    car.add(throttle, inputs=[input_field], threaded=True)

    if not no_cam and (model_path is None or record_on_ai) and not no_tub:
        class RecordingCondition:
            def run(self, throttle_on, throttle_val):
                if model_path is None:
                    return throttle_on and throttle_val > 0
                else:
                    return record_on_ai

        car.add(RecordingCondition(),
                inputs=['user/throttle_on', 'user/throttle'],
                outputs=['user/recording'])

        # if recording ai, push pilot steering into user steering variable, so
        # it gets recorded in the tub
        if model_path is not None and record_on_ai:
            class Identity:
                def run(self, pilot_angle):
                    return pilot_angle
            pilot_to_user_steering = Identity()
            car.add(pilot_to_user_steering, ['pilot/angle'], ['user/angle'])

        # add tub to save data
        inputs = ['cam/image_array', 'user/angle', 'user/throttle',
                  'car/speed', 'car/inst_speed', 'car/lap',
                  'car/accel', 'car/gyro', 'timestamp']
        types = ['image_array', 'float', 'float', 'float', 'float', 'int',
                 'vector', 'vector', 'str']

        # multiple tubs
        tub_handler = TubHandler(path=cfg.DATA_PATH)
        tub = tub_handler.new_tub_writer(inputs=inputs,
                                         types=types,
                                         allow_reverse=False)
        car.add(tub,
                inputs=inputs,
                outputs=["tub/num_records"],
                run_condition='user/recording')

        # add a tub wiper that is triggered by channel 3 on the RC, but only
        # if we don't use channel 3 for switching between ai & manual
        if model_path is None:
            tub_wiper = TubWiper(tub, num_records=car_frequency)
            car.add(tub_wiper, inputs=['user/wiper_on'])

    # run the vehicle
    car.start(rate_hz=car_frequency, max_loop_count=cfg.MAX_LOOPS,
              verbose=verbose)


def calibrate(cfg):
    """
    Construct an auxiliary robotic vehicle from only the RC controllers and
    prints their values. The RC remote usually has a tuning pot for the throttle
    and steering channel. In this loop we run the controllers and simply print
    their values in order to allow centering the RC pwm signals. If there is a
    third channel on the remote we can use it for wiping bad data while
    recording, so we print its values here, too.
    """
    donkey_car = dk.vehicle.Vehicle()

    clock = Timestamp()
    donkey_car.add(clock, outputs=['timestamp'])

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
    # run the vehicle at 5Hz to keep network traffic down
    donkey_car.start(rate_hz=10, max_loop_count=cfg.MAX_LOOPS)


def stream(cfg):
    car = dk.vehicle.Vehicle()
    clock = Timestamp()
    car.add(clock, outputs=['timestamp'])
    hz = 20
    cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                   image_d=cfg.IMAGE_DEPTH, framerate=hz)
    car.add(cam, outputs=['cam/image_array'], threaded=True)
    streamer = FrameStreamer(cfg.PC_HOSTNAME, cfg.FPV_PORT)
    car.add(streamer, inputs=['cam/image_array'], threaded=True)
    car.start(rate_hz=hz, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    config = dk.load_config()
    if args['drive']:
        drive(config,
              use_pid=args['--pid'],
              no_cam=args['--no_cam'],
              model_path=args['--model'],
              web=args['--web'],
              fpv=args['--fpv'],
              no_tub=args['--no_tub'],
              verbose=args['--verbose'])
    elif args['calibrate']:
        calibrate(config)
    elif args['stream']:
        stream(config)
