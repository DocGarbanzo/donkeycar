# """ 
# My CAR CONFIG 

# This file is read by your car application's manage.py script to change the car
# performance

# If desired, all config overrides can be specified here. 
# The update operation will not touch this file.
# """

# VEHICLE
DRIVE_LOOP_HZ = 40
FREQ_REDUCTION_WITH_AI = 0.75

# CAMERA
IMAGE_W = 192
IMAGE_H = 144
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

# STEERING
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 240
STEERING_RIGHT_PWM = 480
STEERING_RC_GPIO = 26

# THROTTLE
THROTTLE_CHANNEL = 0
THROTTLE_FORWARD_PWM = 490
THROTTLE_STOPPED_PWM = 375
THROTTLE_REVERSE_PWM = 280
THROTTLE_RC_GPIO = 20

# DATA WIPER
DATA_WIPER_RC_GPIO = 19

# PID CONTROLLER
PID_P = 0.05
PID_I = 0.3
PID_D = 0.0010  # 0.0005

# ODOMETER
MAX_SPEED = 4.4
ODOMETER_GPIO = 6
TICK_PER_M = 630

# LAP TIMER
LAP_TIMER_GPIO = 16

# TUB SETTINGS
RECORD_DURING_AI = False

# TRAINING
TRAIN_TEST_SPLIT = 0.9
EARLY_STOP_PATIENCE = 10
MAX_EPOCHS = 200
USE_SPEED_FOR_MODEL = True
BATCH_SIZE = 32
CACHE_IMAGES = False

# AUTOPILOT
AI_THROTTLE_MULT = 1.1
