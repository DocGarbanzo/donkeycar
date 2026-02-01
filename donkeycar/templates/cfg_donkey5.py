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
STEERING_LEFT_PWM = 220
STEERING_RIGHT_PWM = 500
STEERING_RC_GPIO = "PICO.BCM.18"

# THROTTLE
THROTTLE_CHANNEL = "PICO.BCM.17"
THROTTLE_FORWARD_PWM = 460
THROTTLE_STOPPED_PWM = 370
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
ODOMETER_GPIO = "PICO.BCM.2"
TICK_PER_M = 75
ODOMETER_USE_PIO = True        # Use PIO counter for long pulses
ODOMETER_FREQUENCY = 20_000    # PIO frequency in Hz (20kHz)

# BATTERY
BATTERY_GPIO = "PICO.BCM.28"

# LAP TIMER
LAP_TIMER_GPIO = 23

# IMU VALUES
IMU_PATH_CORRECTION = (-0.01, 0.0)  # (corr_x, corr_y) per meter
IMU_ACCEL_NORM = 20
IMU_GYRO_NORM = 250
GYRO_Z_INDEX = 2

# =====================================================================
# COURSE ANALYSIS AND SEGMENTATION (Refactored Architecture)
# =====================================================================

"""
Lap Detection Parameters

Controls how individual laps are detected from multi-lap recordings.
Two detection methods are available:

Y-Crossing Method (default):
    Detects lap boundaries when the y-coordinate crosses from negative
    to positive. Works well for closed loops starting/ending near y=0.

Drift Method:
    Uses weighted average reversal point detection. More sophisticated,
    handles courses with drift. Finds the point where the car stops
    approaching the start position and begins moving away.

Parameters:
    y_threshold: Threshold around zero for y-crossing detection (meters)
    min_loop_distance: Minimum distance to travel before considering
        loop closure (meters). Prevents false detections near start.
    max_closure_distance: Maximum distance from start for loop closure
        detection (meters). Only used with drift method.
    weighted_avg_weights: Weights for distance averaging [prev, current, next].
        Used to smooth distance calculations in drift detection.
    reversal_tolerance: Multiplier for detecting when distance starts
        increasing. Values > 1.0 create hysteresis to avoid noise.
    vicinity_window: Maximum number of points to search for reversal
        after entering start vicinity. Limits search to reasonable range.
    time_factor_weight: Weight for time component in reversal scoring.
        Higher values prefer earlier reversals.
    distance_factor_weight: Weight for distance component in reversal
        scoring. Higher values prefer closer reversals.
    min_lap_length: Minimum number of data points per lap. Filters out
        incomplete or erroneous lap detections.
"""
LAP_DETECTION_PARAMS = {
    # Y-Crossing method
    'y_threshold': 0.1,
    'min_loop_distance': 1.0,

    # Drift detection method
    'max_closure_distance': 1.0,
    'weighted_avg_weights': [0.25, 0.5, 0.25],
    'reversal_tolerance': 1.001,
    'vicinity_window': 2000,
    'time_factor_weight': 0.7,
    'distance_factor_weight': 0.3,

    # Common
    'min_lap_length': 50,
}

"""
Mean Course Reconstruction Parameters

Controls how a reference course is computed from multiple laps.
The algorithm resamples all laps onto a common arc-length axis,
computes a weighted average, applies smoothing, and ensures
loop closure continuity.

Algorithm Steps:
1. Resample each lap onto normalized arc-length axis (0 to 1)
2. Compute weighted mean with equal lap contribution
3. Apply Savitzky-Golay smoothing to position
4. Apply moving average smoothing to heading
5. Apply loop closure correction (ensure start/end continuity)
6. Compute cumulative distance along course

Parameters:
    resampling_interval: Distance between resampled points (meters).
        Smaller values = more detail but more computation.
    min_resampling_interval: Minimum allowed interval (meters).
        Safety limit to prevent excessive memory usage.
    position_smoothing_window: Window size for Savitzky-Golay filter
        on position. Must be odd. Larger = smoother but less detail.
    position_polynomial_order: Polynomial order for Savitzky-Golay
        filter. Typically 2 or 3. Higher = follows data more closely.
    heading_smoothing_window: Window size for moving average on heading.
        Circular smoothing accounts for angle wraparound.
    loop_closure_pct: Percentage of course length over which to apply
        closure correction (0-1). Ensures smooth start/end transition.
    outlier_std_threshold: Number of standard deviations for outlier
        detection. Points beyond this are candidates for removal.
    outlier_iterations: Number of outlier removal passes. Multiple
        passes improve robustness to bad data.
"""
MEAN_COURSE_PARAMS = {
    'resampling_interval': 0.1,
    'min_resampling_interval': 0.001,
    'position_smoothing_window': 11,
    'position_polynomial_order': 3,
    'heading_smoothing_window': 5,
    'loop_closure_pct': 0.025,
    'outlier_std_threshold': 2.5,
    'outlier_iterations': 2,
}

"""
Course Segmentation Parameters

Controls how the mean course is divided into geometric segments
(straights, turns, S-curves, chicanes). Multiple segmentation
strategies are available:

Segmentation Strategies:
    Threshold: Detects transitions between straight/turn based on
        curvature threshold. Simple and reliable.
    Extrema: Detects segment boundaries at curvature peaks and valleys
        (apex points). Good for finding turn centers.
    Gradient: Detects boundaries where curvature changes most rapidly
        (entry/exit points). Recommended for most courses.
    Hybrid: Combines threshold + extrema methods for comprehensive
        boundary detection.

Segment Types:
    STRAIGHT: Low curvature sections
    LEFT_TURN: Positive curvature (turning left)
    RIGHT_TURN: Negative curvature (turning right)
    S_CURVE_LR: Left-to-right inflection pattern
    S_CURVE_RL: Right-to-left inflection pattern
    CHICANE: Multiple rapid inflection points

Parameters:
    curvature_window: Number of points for curvature calculation.
        Larger = smoother curvature but less responsive to detail.
    curvature_smoothing_window: Additional smoothing window size.
        Applied after initial curvature calculation.
    straight_curvature_threshold: Threshold for straight vs turn
        classification (radians/meter). Lower = more sensitive.
    min_segment_length: Minimum segment length (meters). Prevents
        tiny segments from noise.
    gradient_prominence: Minimum prominence for gradient-based boundary
        detection. Higher = fewer, more significant boundaries.
    extrema_prominence: Minimum prominence for extrema detection.
        Controls sensitivity to curvature peaks.
    inflection_threshold: Curvature threshold for inflection detection
        (radians/meter). Used in S-curve/chicane classification.
    inflection_chicane_threshold: Number of inflections to classify as
        chicane vs S-curve. Chicanes have more rapid direction changes.
    inflection_scurve_min: Minimum inflections for S-curve detection.
    inflection_scurve_long: Inflection count for longer S-curves.
    scurve_length_threshold: Length threshold (meters) for distinguishing
        short vs long S-curves.
    classification_window: Window size for segment classification
        calculations.
    use_adaptive_threshold: If True, automatically adjust straight
        threshold based on course curvature distribution.
    adaptive_percentile: Percentile of curvature distribution to use
        for adaptive threshold. Lower = straighter classification.
    adaptive_min_threshold: Minimum allowed adaptive threshold (rad/m).
    adaptive_max_threshold: Maximum allowed adaptive threshold (rad/m).
"""
SEGMENTATION_PARAMS = {
    # Curvature calculation
    'curvature_window': 5,
    'curvature_smoothing_window': 21,

    # Boundary detection
    'straight_curvature_threshold': 0.08,  # rad/m
    'min_segment_length': 0.8,             # meters
    'gradient_prominence': 0.1,
    'extrema_prominence': 0.02,

    # Segment classification
    'inflection_threshold': 0.05,          # rad/m
    'inflection_chicane_threshold': 3,
    'inflection_scurve_min': 1,
    'inflection_scurve_long': 2,
    'scurve_length_threshold': 10,         # meters
    'classification_window': 5,

    # Adaptive threshold
    'use_adaptive_threshold': True,
    'adaptive_percentile': 20,
    'adaptive_min_threshold': 0.05,
    'adaptive_max_threshold': 2.0,
}

"""
IMU Path Visualization Parameters

Controls the interactive visualization of recorded IMU paths, mean
courses, and course segmentation. Used by the 'donkey imupath' command.

Visualization Features:
    - Full path scatter plot color-coded by speed
    - Current position marker with navigation
    - Mean course overlay with segment boundaries
    - Segment boundary markers (perpendicular ticks)
    - Interactive controls (time slider, lap selector, etc.)
    - Real-time statistics display

"""


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