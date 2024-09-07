""" 
CAR CONFIG 

This file is read by your car application's manage.py script to change the car
performance. 

"""


import os

# PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

# VEHICLE
DRIVE_LOOP_HZ = 40
MAX_LOOPS = None

# CAMERA
CAMERA_TYPE = "PICAM"   # (PICAM|WEBCAM|CVCAM|CSIC|V4L|D435|MOCK|IMAGE_LIST)
CAMERA_VFLIP = False
CAMERA_HFLIP = False
IMAGE_W = 192
IMAGE_H = 144
IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

# 9865, overrides only if needed, ie. TX2..
PCA9685_I2C_ADDR = 0x40
PCA9685_I2C_BUSNUM = None

# CONTROLLER
USE_RC = True

# STEERING
STEERING_CHANNEL = "PICO.BCM.16"
STEERING_LEFT_PWM = 270
STEERING_RIGHT_PWM = 480
STEERING_RC_GPIO = "PICO.BCM.18"

# THROTTLE
THROTTLE_CHANNEL = "PICO.BCM.17"
THROTTLE_FORWARD_PWM = 490
THROTTLE_STOPPED_PWM = 375
THROTTLE_REVERSE_PWM = 280
THROTTLE_RC_GPIO = "PICO.BCM.19"

# PID CONTROLLER
PID_P = 0.05
PID_I = 0.3
PID_D = 0.0010  # 0.0005

# DATA WIPER
DATA_WIPER_RC_GPIO = 19
CH3_RC_GPIO = "PICO.BCM.20"

# ODOMETER
MAX_SPEED = 4.4
ODOMETER_GPIO = 18
TICK_PER_M = 75

# LAP TIMER
LAP_TIMER_GPIO = 23

# IMU VALUES
IMU_ACCEL_NORM = 20
IMU_GYRO_NORM = 250
GYRO_Z_INDEX = 2


# TRAINING
DEFAULT_AI_FRAMEWORK = 'tensorflow'
DEFAULT_MODEL_TYPE = 'linear'
NN_SIZE = 'R'
CREATE_TF_LITE = True
CREATE_TENSOR_RT = False
BATCH_SIZE = 512
TRAIN_TEST_SPLIT = 0.9
MAX_EPOCHS = 200
SHOW_PLOT = False
VERBOSE_TRAIN = True
USE_EARLY_STOP = True
EARLY_STOP_PATIENCE = 10
MIN_DELTA = .000001
PRINT_MODEL_SUMMARY = True
USE_SPEED_FOR_MODEL = True
CACHE_IMAGES = True
CACHE_POLICY = "ARRAY"
USE_LAP_0 = False

# model transfer options
FREEZE_LAYERS = False
NUM_LAST_LAYERS_TO_TRAIN = 7

# For the categorical model, this limits the upper bound of the learned throttle
MODEL_CATEGORICAL_MAX_THROTTLE_RANGE = 0.8

# RNN or 3D
SEQUENCE_LENGTH = 5

# MEM model
MEM_START_SPEED = 0.5

# Default to fastest quarter or laps
LAP_PCT = [0.5, 0.5, 0.5]
LAP_PCT_L = [0.5, 0.5, 0.5]
LAP_PCT_R = [0.5, 0.5, 0.5]

# Stats setting for lap model
COMPRESS_SESSIONS_FOR_LAP_STATS = True
NUM_BINS_FOR_LAP_STATS = 4

# Augmentations and Transformations
AUGMENTATIONS = ["BRIGHTNESS", "BLUR"]
TRANSFORMATIONS = []
# could be "GAMMANORM" for example
POST_TRANSFORMATIONS = []

# Settings for brightness and blur, use 'BRIGHTNESS' and/or 'BLUR' in
# AUGMENTATIONS
AUG_BRIGHTNESS_RANGE = 0.2  # this is interpreted as [-0.2, 0.2]
AUG_BLUR_RANGE = (0, 3)

# Number of pixels to crop, requires 'CROP' in TRANSFORMATIONS to be set
ROI_CROP_TOP = 50
ROI_CROP_BOTTOM = 0
ROI_CROP_RIGHT = 0
ROI_CROP_LEFT = 0

# For trapezoidal see explanation in augmentations.py, requires 'TRAPEZE' in
# TRANSFORMATIONS to be set
ROI_TRAPEZE_LL = 0
ROI_TRAPEZE_LR = 160
ROI_TRAPEZE_UL = 20
ROI_TRAPEZE_UR = 140
ROI_TRAPEZE_MIN_Y = 60
ROI_TRAPEZE_MAX_Y = 120

# Gamma transformations
GAMMA_NORM_VALUE = 0.3

# "CANNY" Canny Edge Detection tranformation
CANNY_LOW_THRESHOLD = 60    # Canny edge detection low threshold value of intensity gradient
CANNY_HIGH_THRESHOLD = 110  # Canny edge detection high threshold value of intensity gradient
CANNY_APERTURE = 3          # Canny edge detect aperture in pixels, must be odd; choices=[3, 5, 7]

# "BLUR" transformation (not this is SEPARATE from the blur augmentation)
BLUR_KERNEL = 5        # blur kernel horizontal size in pixels
BLUR_KERNEL_Y = None   # blur kernel vertical size in pixels or None for square kernel
BLUR_GAUSSIAN = True   # blur is gaussian if True, simple if False

# "RESIZE" transformation
RESIZE_WIDTH = 160     # horizontal size in pixels
RESIZE_HEIGHT = 120    # vertical size in pixels

# "SCALE" transformation
SCALE_WIDTH = 1.0      # horizontal scale factor
SCALE_HEIGHT = None    # vertical scale factor or None to maintain aspect ratio


# RECORD OPTIONS
RECORD_DURING_AI = False
AUTO_CREATE_NEW_TUB = False

# WEB CONTROL
WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))
WEB_INIT_MODE = "user"

# DRIVING
AI_THROTTLE_MULT = 1.0
AI_ANGLE_MULT = 1.0

# RPi
PI_USERNAME = "pi"
PI_HOSTNAME = "donkeypi.local"

# FPV MONITOR
PC_HOSTNAME = "DirksMacbook.local"
FPV_PORT = 13000

# DonkeyGym
# You will want to download the simulator binary from:
# https://github.com/tawnkramer/donkey_gym/releases/download/vX.X/DonkeySimLinux.zip
# then extract that and modify DONKEY_SIM_PATH.
DONKEY_GYM = False
DONKEY_SIM_PATH = "/home/dirk/DonkeySimLinux/donkey_sim.x86_64"
# when racing on virtual-race-league use "remote", or user "remote" when you
# want to start the sim manually first.
DONKEY_GYM_ENV_NAME = "donkey-generated-track-v0"
# ("donkey-generated-track-v0"|"donkey-generated-roads-v0"|
# "donkey-warehouse-v0"|"donkey-avc-sparkfun-v0")
GYM_CONF = dict(body_style="car01", body_rgb=(96, 96, 96),
                car_name="DocGarbanzo", font_size=40,
                cam_resolution=(IMAGE_H, IMAGE_W, 3),
                cam_config={'img_h': IMAGE_H, 'img_w': IMAGE_W})

GYM_CONF["racer_name"] = "Your Name"
GYM_CONF["country"] = "Place"
GYM_CONF["bio"] = "I race robots."

SIM_HOST = "127.0.0.1"
SIM_ARTIFICIAL_LATENCY = 0

# Save info from Simulator (pln)
SIM_RECORD_LOCATION = True
SIM_RECORD_GYROACCEL = False
SIM_RECORD_VELOCITY = True
SIM_RECORD_LIDAR = False
SIM_RECORD_LAPS = True