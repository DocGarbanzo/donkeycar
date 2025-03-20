# """ 
# My CAR CONFIG 

# This file is read by your car application's manage.py script to change the car
# performance

# If desired, all config overrides can be specified here. 
# The update operation will not touch this file.
# """

# IMU
HAVE_IMU = True
IMU_SENSOR = 'artemis'
IMU_PORT = '/dev/ttyUSB0'
# M_cal = A^-1 * (M - b)
# A is scale factor
IMU_MAG_BIAS = [0.0,0.0,0.0]
IMU_MAG_SCALE_MATRIX = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
IMU_BAUDRATE = 115200
IMU_TIMEOUT = 1

# IMAGE
IMAGE_W = 192
IMAGE_H = 144

# VEHICLE
DRIVE_LOOP_HZ = 40
FREQ_REDUCTION_WITH_AI = 1.0

# TUB SETTINGS
RECORD_DURING_AI = False

# TRAINING
TRAIN_TEST_SPLIT = 0.9
EARLY_STOP_PATIENCE = 10
MAX_EPOCHS = 200
# MIN_DELTA = 0.000001
USE_SPEED_FOR_MODEL = True
# IMU_DIM = 2
NN_SIZE = 'R'
BATCH_SIZE = 1024

# To suppress records when standing or just starting.
def filter_record(record):
    return record.underlying['car/speed'] > 0.5
#TRAIN_FILTER = filter_record


FREEZE_LAYERS = False
NUM_LAST_LAYERS_TO_TRAIN = None  # important to overwrite default of 7
# EXCLUDE_SLOW_LAPS = 0.5
TRAIN_OFFSET = 0
SORT_LAPS_BY = 'lap_time'  # gyro_z, accel_x
# ROI_CROP_TOP = 50
# USE_TENSORBOARD = True
SEQUENCE_LENGTH = 5
SEQUENCE_TRAIN_STEP_SIZE = 1
ENCODER_PATH = 'models/encoder.h5'
OVERWRITE_LATENT = False

# Default to aggressive time, distance, gyro_z quantiles
LAP_PCT = [0.1, 0.25, 0.0]
COMPRESS_SESSIONS_FOR_LAP_STATS = True
NUM_BINS_FOR_LAP_STATS = 4

# AUTOPILOT
AI_THROTTLE_MULT = 1.0

# GYM
DONKEY_GYM_ENV_NAME = "donkey-circuit-launch-track-v0"
GYM_CONF = {"body_style": "car01", "body_rgb": (96, 96, 96),
            "car_name": "DocGarbanzo", "font_size": 40,
            "cam_resolution": (IMAGE_H, IMAGE_W, 3),
            "cam_config": {'img_h': IMAGE_H, 'img_w': IMAGE_W}}
