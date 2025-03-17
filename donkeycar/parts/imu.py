#!/usr/bin/env python3
import math
import time
import board
import busio
import adafruit_mpu6050
import adafruit_bno055
import logging
import imufusion

import serial
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from collections import deque
import time
import threading
import csv
import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt


logger = logging.getLogger(__name__)

SENSOR_MPU6050 = 'mpu6050'
SENSOR_MPU9250 = 'mpu9250'

DLP_SETTING_DISABLED = 0
CONFIG_REGISTER = 0x1A


class IMU:
    '''
    Installation:

    - MPU6050
    sudo apt install python3-smbus
    or
    sudo apt-get install i2c-tools libi2c-dev python-dev python3-dev
    git clone https://github.com/pimoroni/py-smbus.git
    cd py-smbus/library
    python setup.py build
    sudo python setup.py install
    pip install mpu6050-raspberrypi

    - MPU9250
    pip install mpu9250-jmdev

    '''

    def __init__(self, addr=0x68, poll_delay=0.0166, sensor=SENSOR_MPU6050, dlp_setting=DLP_SETTING_DISABLED):
        self.sensortype = sensor
        if self.sensortype == SENSOR_MPU6050:
            from mpu6050 import mpu6050 as MPU6050
            self.sensor = MPU6050(addr)

            if(dlp_setting > 0):
                self.sensor.bus.write_byte_data(self.sensor.address, CONFIG_REGISTER, dlp_setting)

        else:
            from mpu9250_jmdev.registers import AK8963_ADDRESS, GFS_1000, AFS_4G, AK8963_BIT_16, AK8963_MODE_C100HZ
            from mpu9250_jmdev.mpu_9250 import MPU9250

            self.sensor = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=addr,  # In 0x68 Address
                address_mpu_slave=None,
                bus=1,
                gfs=GFS_1000,
                afs=AFS_4G,
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C100HZ)

            if(dlp_setting > 0):
                self.sensor.writeSlave(CONFIG_REGISTER, dlp_setting)
            self.sensor.calibrateMPU6500()
            self.sensor.configure()

        self.accel = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.gyro = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.mag = {'x': 0., 'y': 0., 'z': 0.}
        self.temp = 0.
        self.poll_delay = poll_delay
        self.on = True

    def update(self):
        while self.on:
            self.poll()
            time.sleep(self.poll_delay)
                
    def poll(self):
        try:
            if self.sensortype == SENSOR_MPU6050:
                self.accel, self.gyro, self.temp = self.sensor.get_all_data()
            else:
                from mpu9250_jmdev.registers import GRAVITY
                ret = self.sensor.getAllData()
                self.accel = { 'x' : ret[1] * GRAVITY, 'y' : ret[2] * GRAVITY, 'z' : ret[3] * GRAVITY }
                self.gyro = { 'x' : ret[4], 'y' : ret[5], 'z' : ret[6] }
                self.mag = { 'x' : ret[13], 'y' : ret[14], 'z' : ret[15] }
                self.temp = ret[16]
        except:
            print('failed to read imu!!')
            
    def run_threaded(self):
        return \
            [a-z for a, z in zip(list(self.accel.values()), self.accel_zero)], \
            [g-z for g, z in zip(list(self.gyro.values()), self.gyro_zero)]

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.on = False


class Mpu6050Ada:
    def __init__(self, sample_rate=100):
        logger.info("Creating Adafruit Mpu6050 ...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
        self.mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
        self.mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_500_DPS
        # set filter to 44Hz data smoothing
        self.mpu.filter_bandwidth = 3
        self.accel_zero = None
        self.gyro_zero = None
        self.accel = np.zeros(3)
        self.gyro = np.zeros(3)
        self.on = True
        self.pos = np.zeros(3)
        self.speed = np.zeros(3)
        self.speed_drift = np.zeros(3)
        self.time = None
        self.path = [] # [(self.time, *self.pos)]
        self.sample_rate = sample_rate
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            # imufusion.CONVENTION_NWU,
            imufusion.CONVENTION_ENU,
            0.5,  # gain
            2000,  # gyroscope range
            10,  # acceleration rejection
            10,  # magnetic rejection
            5 * self.sample_rate,  # recovery trigger period = 5 seconds
        )
        self.offset = imufusion.Offset(self.sample_rate)
        self.matrix = None
        self.euler = None
        self.lin_accel = None
        self.accel_norm = 0
        self.calibrate()

    def calibrate(self):
        logger.info('Calibrating Mpu6050 ...')
        num_loops = 200
        gyro = np.zeros(3)
        accel = np.zeros(3)
        accel_norm = 0
        # run w/o doing anything:
        for _ in range(num_loops//2):
            tmp = self.mpu.gyro
            tmp = self.mpu.acceleration
        tic = time.time()
        for _ in range(num_loops):
            gyro += self.mpu.gyro
            accel += self.mpu.acceleration
            accel_norm += np.linalg.norm(self.mpu.acceleration)
            toc = time.time()
            if toc - tic < 1 / self.sample_rate:
                time.sleep(1 / self.sample_rate - (toc - tic))
            tic = time.time()
        self.gyro_zero = gyro / num_loops
        self.accel_zero = accel / num_loops
        self.accel_norm = accel_norm / num_loops
        logger.info(f'Initial acceleration: {self.accel_zero}, '
                    f'norm: {self.accel_norm}')
        while self.ahrs.flags.initialising:
            self.poll()
            toc = time.time()
            if toc - tic < 1 / self.sample_rate:
                time.sleep(1 / self.sample_rate - (toc - tic))
            tic = time.time()
        self.time = time.time()
        logger.info('Calibrated the Imu algorithm...')
        self.pos = np.zeros(3)
        self.speed = np.zeros(3)
        self.path.clear()
        self.time = tic
        logger.info(f'Determined speed drift {self.speed_drift}.  - Mpu6050 '
                    f'calibrated')

    def update(self):
        while self.on:
            self.poll()

    def poll(self):
        alpha = 1.0
        new_time = time.time()
        if self.time is None:
            self.time = new_time
        dt = new_time - self.time
        gyro = np.array(self.mpu.gyro) - self.gyro_zero
        # convert from radians to degrees
        gyro_degree = np.array(gyro) * 180 / math.pi
        adj_gyro = self.offset.update(gyro_degree)
        self.accel = ((1-alpha) * self.accel
                      + alpha * np.array(self.mpu.acceleration))
        self.ahrs.update_no_magnetometer(adj_gyro, self.accel/self.accel_norm, dt)
        if not self.ahrs.flags.initialising:
            self.euler = self.ahrs.quaternion.to_euler()
            self.matrix = self.ahrs.quaternion.to_matrix()
            accel_ignore = self.ahrs.internal_states.accelerometer_ignored
            if not accel_ignore:
                self.lin_accel = self.accel_norm * self.ahrs.earth_acceleration
                delta_v = self.lin_accel * dt
                self.speed += delta_v
            self.pos += (self.speed - self.speed_drift) * dt
            self.path.append((self.time, *self.pos, np.linalg.norm(self.speed)))
        self.time = new_time

    def run(self):
        self.poll()
        return self.euler, self.lin_accel, self.matrix

    def run_threaded(self):
        return self.euler, self.lin_accel, self.matrix

    def shutdown(self):
        self.on = False
        df = pd.DataFrame(columns=['t', 'x', 'y', 'z', 'v'], data=self.path)
        df.to_csv('imu.csv', index=False)
        logger.info('Mpu6050 shutdown - saved path to imu.csv')


class BNO055Ada:
    def __init__(self, alpha=1.0, record_path=False):
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.last_val = 0xFFFF
        self.sensor.offsets_accelerometer = (41, -116, -26)
        self.sensor.offsets_gyroscope = (-1, -1, -1)
        self.sensor.offsets_magnetometer = (-176, 196, 17)
        self.speed = np.zeros(3)
        self.pos = np.zeros(3)
        self.accel = np.zeros(3) # np.array(self.sensor.linear_acceleration)
        self.path = []
        self.time = None
        self.on = True
        # euler angles are in z, y, x order in the sensor
        self.euler = np.array(self.sensor.euler[::-1])
        self.alpha = alpha
        self.record_path = record_path

    def temperature(self):
        result = self.sensor.temperature
        if abs(result - self.last_val) == 128:
            result = self.sensor.temperature
            if abs(result - self.last_val) == 128:
                return 0b00111111 & result
        self.last_val = result
        return result

    def poll(self):
        new_time = time.time()
        if self.time is None:
            self.time = new_time
        dt = new_time - self.time
        gyro = np.array(self.sensor.gyro)
        self.euler *= (1.0 - self.alpha)
        # euler angles are in z, y, x order in the sensor
        self.euler += self.alpha * np.array(self.sensor.euler[::-1])
        self.accel *= (1.0 - self.alpha)
        self.accel += self.alpha * np.array(self.sensor.linear_acceleration)
        if self.record_path:
            delta_v = self.accel * dt
            self.speed += delta_v
            self.pos += self.speed * dt
            self.path.append((self.time, *self.pos, np.linalg.norm(self.speed)))
        self.time = new_time

    def update(self):
        while self.on:
            self.poll()

    def run_threaded(self):
        return self.euler, self.accel

    def run(self):
        self.poll()
        return self.euler, self.accel

    def shutdown(self):
        self.on = False
        if self.record_path:
            df = pd.DataFrame(columns=['t', 'x', 'y', 'z', 'v'], data=self.path)
            df.to_csv('imu.csv', index=False)
            logger.info('BNO055050 shutdown - saved path to imu.csv')

class ArtemisOpenLog:
    def __init__(self, port, baudrate, timeout):
        assert(isinstance(port, str)), "Port must be a valid string input"
        assert(isinstance(baudrate, int) and baudrate > 0), "Baudrate must be valid int"
        assert((isinstance(timeout, int) or isinstance(timeout, float)) and timeout > 0), "timeout time must be valid number greater than 0" 
        self.serial_port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.read_imu_thread = None

        self.gyro_offset = None
        self.accel_offset = None

        # Acceleration is in milli g
        self.accel = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.accel_x = deque(maxlen=100)
        self.accel_y = deque(maxlen=100)
        self.accel_z = deque(maxlen=100)
        # Gyroscope data is in degrees per second
        self.gyro = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.gyro_x = deque(maxlen=100)
        self.gyro_y = deque(maxlen=100)
        self.gyro_z = deque(maxlen=100)
        self.temp = 0.0

        self.yaw = 0.0
        self.yaw_values = deque(maxlen=100)  # Deque to store yaw values
        self.ekf_state = np.zeros(2)  # [yaw, yaw_rate]
        self.ekf_covariance = np.eye(2)  # Initial covariance matrix

        self.connect()
    
    def connect(self):
        """
        Sets up the serial communication of the OpenLog IMU
        via UART with the given port, baudrate, and the timeout.

        :param port: The serial port to communicate through
        :param baudrate: The baudrate of the serial communcation
        :param timeout: The timeout
        """
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            logger.info(f"Connected to {self.serial_port}")
        except serial.SerialException as e:
            logger.error(f"Error: {e}")
        except KeyboardInterrupt:
            logger.info("\nExiting...")
            self.shutdown()
            

    def read_imu_data(self, log_data: bool = None, log_type: str = None):
        """
        Read the data from the ICM20948 IMU
        that is on the Artemis OpenLog.

        :param log_data: Boolean to determine if we want to also log the data in matplotlib figure.
        :param log_type: String to determine what to log on the matplotlib figure if we want to plot.
        """
        assert(isinstance(log_data, bool) or log_data is None), "log_data must be True or False"
        assert((isinstance(log_type, str) or log_type is None) and (log_type == "accel" or log_type == "gyro" or log_type is None)), "log_type must be 'accel' or 'gyro'"

        if self.ser is None:
            logger.error("Serial connection not initialized")
            return

        while self.running:

            #with open('imu_data_threaded.csv', 'w', newline='') as fp_imu:
                
            try:
                # Artemis OpenLog already records data in CSV format for us
                # Look at datasheet for Artemis OpenLog Sparkfun to see what data
                # points are stored in which indices in the data array when reading 
                data = self.ser.readline().decode('utf-8').strip().split(",")
                str_vals = ['rtcDate', 'rtcTime', 'aX', 'aY', 'aZ', 'gX', 'gY', 'gZ', 'mX', 'mY', 'mZ', 'imu_degC', 'output_Hz']
                if len(data) >= 12:

                    data_array = data[0:len(data)-1]
                    
                    print(data_array)

                    # This if statement is critical for plotting the data in real time.
                    if data_array != str_vals:
                        # Apply accel offset if available
                        if self.accel_offset:
                            self.accel = {
                                'x' : float(data_array[2]) * 0.00981 - self.accel_offset['x'],
                                'y' : float(data_array[3]) * 0.00981 - self.accel_offset['y'],
                                'z' : float(data_array[4]) * 0.00981 - self.accel_offset['z']
                            }
                        else:
                            self.accel = {
                                'x' : float(data_array[2]) * 0.00981,
                                'y' : float(data_array[3]) * 0.00981,
                                'z' : float(data_array[4]) * 0.00981
                            }
                        # For logging the previous 100 points if necessary
                        self.accel_x.append(self.accel['x'])
                        self.accel_y.append(self.accel['y'])
                        self.accel_z.append(self.accel['z'])
                        
                        # Apply gyro offset if available
                        if self.gyro_offset:
                            raw_gyro = { 
                                'x' : float(data_array[5]) - self.gyro_offset['x'], 
                                'y' : float(data_array[6]) - self.gyro_offset['y'], 
                                'z' : float(data_array[7]) - self.gyro_offset['z'] 
                            }
                        else:
                            raw_gyro = { 
                                'x' : float(data_array[5]), 
                                'y' : float(data_array[6]), 
                                'z' : float(data_array[7]) 
                            }
                        # For logging the previous 100 points if necessary
                        self.gyro_x.append(self.gyro['x'])
                        self.gyro_y.append(self.gyro['y'])
                        self.gyro_z.append(self.gyro['z'])
                        self.temp = float(data_array[11])


                        # Kalman Filter parameters to smoothen the gyroscope data
                        # Depending on your system, may need to tune Q and R by trial-and-error
                        # Found the below parameters to work really well
                        x = np.zeros(3) 
                        P = np.eye(3)  
                        Q = np.eye(3) * 0.001  
                        R = np.eye(3) * 10 
                        z = np.array([raw_gyro['x'], raw_gyro['y'], raw_gyro['z']])
                        x, P = self.kalman_filter_gyro(x, P, z, Q, R)

                        self.gyro = {
                            'x': x[0],
                            'y': x[1],
                            'z': x[2]
                        }

                        self.gyro_x.append(self.gyro['x'])
                        self.gyro_y.append(self.gyro['y'])
                        self.gyro_z.append(self.gyro['z'])
                        self.temp = float(data_array[11])

                        dt = 0.01  # Assuming a fixed time step for simplicity
                        mag_yaw = np.arctan2(float(data_array[9]), float(data_array[8]))
                        # Use the EKF sensor fusion between gyroscope
                        # and magnometer to get the yaw angle 
                        self.ekf_gyro_mag(dt, mag_yaw)
                        self.yaw = self.ekf_state[0]
                        self.yaw_values.append(self.yaw)

            except Exception as e:
                logger.error(f"Error reading data from Artemis IMU: {e}")
                time.sleep(0.1)

    def start_imu_threading(self):
        """
        Starts the thread for reading the IMU data to
        run in the backround while also being connected.
        """
        if self.running and self.read_imu_thread is not None:
            logger.info("IMU data reading thread is already running")
            return
        if self.read_imu_thread is None or not self.read_imu_thread.is_alive():
            self.running = True
            self.read_imu_thread = threading.Thread(target=self.read_imu_data, daemon=True)
            self.read_imu_thread.start()
            logger.info("Started IMU data reading thread")

    def stop_imu_threading(self):
        """
        Stops the thread for reading the IMU data.
        """
        self.running = False
        if self.read_imu_thread:
            self.read_imu_thread.join()
            logger.info("Stopped IMU data readingt thread")

    def ekf_gyro_mag(self, dt, mag_yaw):
        """
        Extended Kalman Filter to predict and update the state based on gyroscope and magnetometer data.

        :param dt: Time step between predictions.
        :param mag_yaw: Yaw angle from the magnetometer.
        """
        assert((isinstance(dt, int) or isinstance(dt, float)) and dt > 0), "dt must be a valid number for EKF"
        assert(isinstance(mag_yaw, float)), "The yaw angle must be a valid float"

        # Prediction step
        F = np.array([[1, dt], [0, 1]]) 
        Q = np.array([[0.01, 0], [0, 0.01]]) 

        self.ekf_state = np.dot(F, self.ekf_state)
        self.ekf_covariance = np.dot(np.dot(F, self.ekf_covariance), F.T) + Q

        # Update/Correct the yaw angle 
        H = np.array([[1, 0]])
        R = np.array([[0.1]]) 

        y = mag_yaw - np.dot(H, self.ekf_state)
        S = np.dot(np.dot(H, self.ekf_covariance), H.T) + R 
        K = np.dot(np.dot(self.ekf_covariance, H.T), np.linalg.inv(S))

        self.ekf_state = self.ekf_state + np.dot(K, y)
        self.ekf_covariance = np.dot((np.eye(2) - np.dot(K, H)), self.ekf_covariance)

    def kalman_filter_gyro(self, x, P, z, Q, R):
        """
        Applies 3D Kalman Filter to smoothen the gyroscope data. Must smoothen
        the gyroscope data to get better sensor fusion with the GPS.

        :param x: The Initial state that we want to predict (3x1 vector)
        :param P: The initial uncertainty covariance of our initial data readings (3x3 matrix)
        :param z: The noisy gyroscope data to smoothen out (3x1 vector)
        :param Q: Process noise covariance (3x3 matrix)
        :param R: Measurement noise covariance (3x3 matrix)

        :return: Updated/corrected gyroscope data and covariance uncertainty (x_updated, P_updated)
        """
        assert(isinstance(x, np.ndarray) and x.shape == (3,)), "x must be a 3x1 numpy array"
        assert(isinstance(P, np.ndarray) and P.shape == (3, 3)), "P must be a 3x3 numpy array"
        assert(isinstance(z, np.ndarray) and z.shape == (3,)), "z must be a 3x1 numpy array"
        assert(isinstance(Q, np.ndarray) and Q.shape == (3, 3)), "Q must be a 3x3 numpy array"
        assert(isinstance(R, np.ndarray) and R.shape == (3, 3)), "R must be a 3x3 numpy array"

        # Predict the new gyroscope data
        F = np.eye(3)
        x_pred = np.dot(F, x)
        P_pred = np.dot(np.dot(F,P), F.T) + Q

        # Update/Correct the raw gyroscope data
        H = np.eye(3)
        y_residual = z - np.dot(H, x_pred)
        S = np.dot(np.dot(H, P_pred), H.T) + R
        K_gain = np.dot(np.dot(P_pred, H.T), np.linalg.inv(S))
        x_updated = x_pred + np.dot(K_gain, y_residual)
        P_updated = P_pred - np.dot(np.dot(K_gain, H), P_pred)

        return x_updated, P_updated

    def low_pass_filter_gyro(self, cutoff, freq, data, order=5):
        """
        Applies a lowpass filter for the imu gyro data.

        :param cutoff: The cutoff frequency for the low-pass filter (in Hz).  
        :param freq: The sampling frequency of the data (in Hz).  
        :param data: A list of gyroscope data to be filtered.   
        :param order: The order of the Butterworth filter (higher values give a steeper filter).
            
        :return: The filtered gyroscope data with high-frequency noise removed.
        """
        assert((isinstance(cutoff, int) or isinstance(cutoff, float)) and cutoff > 0), "The cutoff for filter must be valid number"
        assert((isinstance(freq, int) or isinstance(freq, float)) and freq > 0), "The frequency for filter must be valid number"
        assert(isinstance(data, list) and len(data) > 0), "The input data must be a valid list"
        assert(isinstance(order, int) and order > 0), "Order must be an int greater than 5"
        nyquist = 0.5 * freq
        normal_cuttoff = cutoff / nyquist
        b, a = butter(order, normal_cuttoff, btype='low', analog=False)
        return filtfilt(b, a, data)

    def poll(self):
        """
        Calls the read_imu_data() method and polls
        the IMU to read the data that it records.
        Non-threaded polling.
        """
        self.read_imu_data()

    def run(self, sleep_time):
        """
        Calls poll() method to poll, read, and record
        data from the IMU on Artemis OpenLog every 
        sleep_time interval. Non-threaded.

        :param sleep_time: The time interval (in seconds) between each IMU poll()
        """
        assert((isinstance(sleep_time, int) or isinstance(sleep_time, float)) and sleep_time > 0), "Sleep time must be greater than 0" 
        self.poll()
        time.sleep(sleep_time)

    def shutdown(self):
        """
        Terminates communcation and the
        serial port on the Artemis OpenLog.
        """
        self.stop_imu_threading()
        if self.ser:
            self.ser.close()
            logger.info("Closed serial connection")


#---------- Free Functions ----------#

def calibrate_artemis(imu: ArtemisOpenLog, period: int, calibrate_gyro: bool = True, calibrate_accel: bool = True):
    """
    Runs the IMU for a given period time to calculate
    and set the gyro and/or accel offset of the Artemis.

    :param imu: The ArtemisOpenLog IMU to run for a given period of time.
    :param period: The given amount of time to collect samples.
    :param calibrate_gyro: Boolean to determine if gyro calibration is needed.
    :param calibrate_accel: Boolean to determine if accel calibration is needed.

    :return: The gyro and/or accel offset/bias that will be used to calibrate the IMU.
    """
    assert(isinstance(imu, ArtemisOpenLog)), "The imu must be an ArtemisOpenLog object"
    assert(isinstance(period, int) and period > 0), "The period must be a valid int"
    assert(isinstance(calibrate_gyro, bool)), "calibrate_gyro must be a boolean"
    assert(isinstance(calibrate_accel, bool)), "calibrate_accel must be a boolean"
    
    imu.start_imu_threading()

    gyro_x_samples = []
    gyro_y_samples = []
    gyro_z_samples = []
    accel_x_samples = []
    accel_y_samples = []
    accel_z_samples = []

    start_time = time.time()
    while period > time.time() - start_time:
        if calibrate_gyro and imu.gyro_x and imu.gyro_y and imu.gyro_z:
            gyro_x_samples.append(imu.gyro['x'])
            gyro_y_samples.append(imu.gyro['y'])
            gyro_z_samples.append(imu.gyro['z'])
        if calibrate_accel and imu.accel_x and imu.accel_y and imu.accel_z:
            accel_x_samples.append(imu.accel['x'])
            accel_y_samples.append(imu.accel['y'])
            accel_z_samples.append(imu.accel['z'])
        time.sleep(0.01)  # Small delay to avoid excessive CPU usage

    imu.stop_imu_threading()

    if calibrate_gyro:
        gyro_offset = {
            'x': np.mean(gyro_x_samples),
            'y': np.mean(gyro_y_samples),
            'z': np.mean(gyro_z_samples)
        }
        imu.gyro_offset = gyro_offset
    else:
        gyro_offset = None

    if calibrate_accel:
        accel_offset = {
            'x': np.mean(accel_x_samples),
            'y': np.mean(accel_y_samples),
            'z': np.mean(accel_z_samples)
        }
        imu.accel_offset = accel_offset
    else:
        accel_offset = None

    return gyro_offset, accel_offset

def log_data(imu: ArtemisOpenLog, interval_time, log_type: str):
    """
    Logs the recorded data from the given imu object parameter
    into a matplotlib plot in real time for debugging purposes. 

    :param imu: The imu object to plot data fram
    :param interval_time: The time between each data log in the matplotlib
    :param log_type: The type to plot against time. "accel" or "gyro" strings.

    """
    assert(isinstance(imu, ArtemisOpenLog)), "imu parameter must be an ArtemisOpenLog object"
    assert((isinstance(interval_time, int) or isinstance(interval_time, float)) and interval_time > 0), "Interval time must be a valid number"
    assert(isinstance(log_type, str) and (log_type == 'accel' or log_type == 'gyro')), "log_type argument must be a string of either 'accel' or 'gyro'"

    fig, ax = plt.subplots()
    ax.set_xlabel("Time")
    ax.set_ylim(-10, 10) 

    if log_type == "accel":
        ax.set_ylabel("Acceleration (m/s²)")
        line_ax, = ax.plot([], [], label="aX", color="r")
        line_ay, = ax.plot([], [], label="aY", color="g")
        line_az, = ax.plot([], [], label="aZ", color="b")
        ax.legend()

        def update_plot(_):
            line_ax.set_data(range(len(imu.accel_x)), imu.accel_x)
            line_ay.set_data(range(len(imu.accel_y)), imu.accel_y)
            line_az.set_data(range(len(imu.accel_z)), imu.accel_z)

            ax.set_xlim(0, len(imu.accel_x))
            return line_ax, line_ay, line_az
    
    elif log_type == "gyro":
        ax.set_ylabel("Gyroscope (deg/s)")
        line_gx, = ax.plot([], [], label="gX", color="r")
        line_gy, = ax.plot([], [], label="gY", color="g")
        line_gz, = ax.plot([], [], label="gZ", color="b")
        ax.legend()

        def update_plot(_):
            line_gx.set_data(range(len(imu.gyro_x)), imu.gyro_x)
            line_gy.set_data(range(len(imu.gyro_y)), imu.gyro_y)
            line_gz.set_data(range(len(imu.gyro_z)), imu.gyro_z)

            ax.set_xlim(0, len(imu.gyro_x)) 
            return line_gx, line_gy, line_gz

    ani = animation.FuncAnimation(fig, update_plot, interval=interval_time, cache_frame_data=False)
    plt.show()

import multiprocessing
from multiprocessing import Process, Value
import time


def plot(mlist, limit=5, update_freq=0, running=Value('i', 1)):
    plt.style.use('dark_background')
    plt.ion()
    fig = plt.figure()
    ax = plt.axes(xlim=(-limit, limit), ylim=(-limit, limit))
    line, = ax.plot([], [], lw=2)
    fig.canvas.draw()
    tic = time.time()
    count = 0
    while running.value == 1:
        try:
            np_path = np.array(mlist)
            line.set_xdata(np_path[:, 1])
            line.set_ydata(np_path[:, 2])
            fig.canvas.draw()
            fig.canvas.flush_events()
        except Exception as e:
            pass
        toc = time.time()
        dtime = toc - tic
        if update_freq and dtime < 1.0 / update_freq:
            time.sleep(1.0 / update_freq - dtime)
        tic = toc
        count += 1
    print(f"Ran plot job for {count} iterations")


class PathPlotter:
    def __init__(self, update_freq=0, limit=5):
        self.manager = multiprocessing.Manager()
        self.mlist = self.manager.list()
        self.update_freq = update_freq
        self.plot_proc = None
        self.running = Value('i', 1)
        self.limit = limit

    def start(self):
        self.plot_proc = Process(
            target=plot, args=(self.mlist, self.limit,
                               self.update_freq, self.running))
        self.plot_proc.start()

    def stop(self):
        self.running.value = 0
        self.plot_proc.join()

    def append(self, x):
        if len(x) != 2:
            raise ValueError("Input must be a 2-sequence")
        self.mlist.append((time.time(), *x))


if __name__ == "__main__":
    import sys
    from sys import stdout
    import threading
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=4, sign='+', floatmode='fixed',
                        suppress=True)
    count = 0
    sample_rate = 40
    mpu = BNO055Ada(alpha=0.5)
    t = threading.Thread(target=mpu.update, daemon=True)
    t.start()
    pp = PathPlotter(update_freq=20, limit=2)
    pp.start()
    print('Go!')
    start = time.time()
    tic = start
    while True:
        try:
            euler, accel = mpu.run_threaded()
            #out_str = f"\reuler: " + f",".join(f"{x:+5.3f}" for x in matrix)
            out_str = f"\reu = " + \
                np.array2string(euler, precision=1, separator=',',
                                sign='+', floatmode='fixed',
                                suppress_small=True).replace('\n', '')
            out_str += \
                ' a = ' + \
                np.array2string(accel, precision=2, separator=',',
                                sign='+', floatmode='fixed',
                                suppress_small=True).replace('\n', '')
            stdout.write(out_str)
            stdout.flush()
            pos = mpu.pos[:2]
            pp.append(pos)
            toc = time.time()
            if toc - tic < 1 / sample_rate:
                time.sleep(1 / sample_rate - (toc - tic))
            tic = time.time()
            count += 1
        except KeyboardInterrupt:
            pp.stop()
            mpu.shutdown()
            stdout.write("\n")
            break
    print(f'Effective sampling time: {(time.time() - start) * 1000/count:.2f} '
          f'ms')
    inp = input("Press enter to stop plotting")
    sys.exit(0)

    iter = 0
    import sys
    sensor_type = SENSOR_MPU6050
    dlp_setting = DLP_SETTING_DISABLED
    if len(sys.argv) > 1:
        sensor_type = sys.argv[1]
    if len(sys.argv) > 2:
        dlp_setting = int(sys.argv[2])

    mpu = IMU(sensor=sensor_type)
    while iter < 100:
        data = mpu.run()
        print(data)
        time.sleep(0.1)
        iter += 1

