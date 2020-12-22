"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXMAPLE
-----------
import dk
cfg = dk.load_config(config_path='~/mycar/config.py')
print(cfg.CAMERA_RESOLUTION)

"""


import os

# PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

# VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000
MAX_SPEED = 4.0

# CAMERA
CAMERA_RESOLUTION = (144, 192)  #(height, width)
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

# STEERING
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 240
STEERING_RIGHT_PWM = 440
STEERING_RC_GPIO = 26

# THROTTLE
THROTTLE_CHANNEL = 0
THROTTLE_FORWARD_PWM = 440
THROTTLE_STOPPED_PWM = 370
THROTTLE_REVERSE_PWM = 300
THROTTLE_RC_GPIO = 20

# DATA WIPER
DATA_WIPER_RC_GPIO = 19

# TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8
PATIENCE = 10
EPOCHS = 200

TUB_PATH = os.path.join(CAR_PATH, 'tub')  # if using a single tub

# ROPE.DONKEYCAR.COM
ROPE_TOKEN = "GET A TOKEN AT ROPE.DONKEYCAR.COM"
