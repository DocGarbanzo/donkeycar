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
import ekf_imu


logger = logging.getLogger(__name__)

SENSOR_MPU6050 = 'mpu6050'
SENSOR_MPU9250 = 'mpu9250'
SENSOR_ARTEMIS = 'artemis'

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
    def __init__(self, port, baudrate, timeout, mag_bias=None, mag_scale_matrix=None, sample_rate=40):
        assert(isinstance(port, str)), "Port must be a valid string input"
        assert(isinstance(baudrate, int) and baudrate > 0), "Baudrate must be valid int"
        assert((isinstance(timeout, int) or isinstance(timeout, float)) and timeout > 0), "Timeout must be a valid number greater than 0"
        self.ser = None
        self.accel = {'x': 0, 'y': 0, 'z': 0}
        self.gyro = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_smooth = {'x': 0, 'y': 0, 'z': 0}
        self.euler = {'x': 0, 'y': 0, 'z': 0}
        self.mag = {'x': 0, 'y': 0, 'z': 0}
        self.mag_bias = np.array(mag_bias) if mag_bias is not None else np.zeros(3)
        self.mag_scale_matrix = np.array(mag_scale_matrix) if mag_scale_matrix is not None else np.eye(3)
        self.accel_x = deque(maxlen=100)
        self.accel_y = deque(maxlen=100)
        self.accel_z = deque(maxlen=100)
        self.gyro_x = deque(maxlen=100)
        self.gyro_y = deque(maxlen=100)
        self.gyro_z = deque(maxlen=100)
        self.gyro_x_s = deque(maxlen=100)
        self.gyro_y_s = deque(maxlen=100)
        self.gyro_z_s = deque(maxlen=100)
        self.pos = np.zeros(3)
        self.speed = np.zeros(3)
        self.speed_drift = np.zeros(3)
        self.path = []
        self.time = None
        self.sample_rate = sample_rate

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            logger.info(f"Connected to {port}")
        except serial.SerialException as e:
            logger.error(f"Error: {e}")
        except KeyboardInterrupt:
            logger.info("\nExiting...")

    def calibrate(self):
        logger.info('Calibrating Artemis IMU ...')
        num_loops = 200
        gyro = np.zeros(3)
        accel = np.zeros(3)
        accel_norm = 0
        mag_samples = []  # Collect raw magnetometer readings
        
        # Warm-up phase: discard initial readings
        for _ in range(num_loops // 2):
            self.poll()
            time.sleep(1.0 / self.sample_rate)
        
        tic = time.time()
        for _ in range(num_loops):
            self.poll()
            # Accumulate gyroscope readings
            gyro += np.array([self.gyro['x'], self.gyro['y'], self.gyro['z']])
            # Accumulate accelerometer readings
            accel += np.array([self.accel['x'], self.accel['y'], self.accel['z']])
            # Accumulate the norm of the acceleration
            accel_norm += np.linalg.norm([self.accel['x'], self.accel['y'], self.accel['z']])
            # Save raw magnetometer reading
            mag_samples.append(np.array([self.mag['x'], self.mag['y'], self.mag['z']]))
            
            toc = time.time()
            if toc - tic < 1.0 / self.sample_rate:
                time.sleep(1.0 / self.sample_rate - (toc - tic))
            tic = time.time()
        
        # Compute biases for gyroscope and accelerometer
        self.gyro_bias = gyro / num_loops
        self.accel_bias = accel / num_loops
        self.accel_norm = accel_norm / num_loops
        logger.info(f'Initial acceleration bias: {self.accel_bias}, norm: {self.accel_norm}')
        
        # Magnetometer calibration:
        mag_array = np.array(mag_samples)
        # Hard iron correction: use the median for each axis
        self.mag_bias = np.median(mag_array, axis=0)
        # Bias-correct the magnetometer readings
        corrected = mag_array - self.mag_bias
        # Calculate half-range for each axis
        max_vals = np.max(corrected, axis=0)
        min_vals = np.min(corrected, axis=0)
        half_range = (max_vals - min_vals) / 2.0
        # Average half-range as the target sphere radius
        avg_half_range = np.mean(half_range)
        # Scale factors: target divided by each axis' half-range
        scale_factors = avg_half_range / half_range
        # Build the diagonal soft iron correction matrix
        self.mag_scale_matrix = np.diag(scale_factors)
        
        logger.info(f'Magnetometer bias: {self.mag_bias}')
        logger.info(f'Magnetometer scale matrix: {self.mag_scale_matrix}')
        
        # Reset kinematic variables
        self.pos = np.zeros(3)
        self.speed = np.zeros(3)
        self.path.clear()
        self.time = time.time()
        
        logger.info('Artemis IMU calibrated.')

    def poll(self):
        """
        Reads the IMU data from the serial port and updates the internal fields.
        """
        data = self.ser.readline().decode('utf-8').strip().split(",")
        try:
            self.accel = {
                'x': float(data[2]) * 0.00981,
                'y': float(data[3]) * 0.00981,
                'z': float(data[4]) * 0.00981
            }
            self.gyro = {
                'x': float(data[5]),
                'y': float(data[6]),
                'z': float(data[7])
            }
            raw_mag = np.array([float(data[8]), float(data[9]), float(data[10])])
            # Apply calibration to magnetometer data:
            calibrated_mag = np.dot(self.mag_scale_matrix, raw_mag - self.mag_bias)
            self.mag = {
                'x': calibrated_mag[0],
                'y': calibrated_mag[1],
                'z': calibrated_mag[2]
            }
            self.gyro_x.append(self.gyro['x'])
            self.gyro_y.append(self.gyro['y'])
            self.gyro_z.append(self.gyro['z'])
    
            # EKF sensor fusion for orientation
            ekf = ekf_imu.EKF_IMU(dt=0.01)
            g = np.array(list(self.gyro.values()))
            ekf.predict(g)
            a = np.array(list(self.accel.values()))
            m = np.array(list(self.mag.values()))
            ekf.update(a, m)
            euls = ekf.get_euler_angles()
            self.euler = {
                'x': float(euls[0]),
                'y': float(euls[1]),
                'z': float(euls[2])
            }
    
            # Smooth gyroscope data using a 3D Kalman filter
            x = np.zeros(3)
            P = np.eye(3)
            Q = np.eye(3) * 0.001
            R = np.eye(3) * 10
            z = np.array([self.gyro['x'], self.gyro['y'], self.gyro['z']])
            x, P = self.kalman_filter_gyro(x, P, z, Q, R)
            self.gyro_smooth = {
                'x': x[0],
                'y': x[1],
                'z': x[2]
            }
            self.gyro_x_s.append(self.gyro_smooth['x'])
            self.gyro_y_s.append(self.gyro_smooth['y'])
            self.gyro_z_s.append(self.gyro_smooth['z'])
            
        except IndexError:
            logger.error("Waiting for IMU data to parse")
        except ValueError:
            logger.error("Waiting for IMU data to cast to floats")
    
    def kalman_filter_gyro(self, x, P, z, Q, R):
        """
        Applies a 3D Kalman Filter to smooth the gyroscope data.
    
        :param x: Initial state (3x1 vector) as a NumPy array.
        :param P: Initial covariance matrix (3x3).
        :param z: Measured gyro data (3x1 vector) as a NumPy array.
        :param Q: Process noise covariance (3x3).
        :param R: Measurement noise covariance (3x3).
        :return: Tuple (x_updated, P_updated) with updated state and covariance.
        """
        assert(isinstance(x, np.ndarray) and x.shape == (3,))
        assert(isinstance(P, np.ndarray) and P.shape == (3, 3))
        assert(isinstance(z, np.ndarray) and z.shape == (3,))
        assert(isinstance(Q, np.ndarray) and Q.shape == (3, 3))
        assert(isinstance(R, np.ndarray) and R.shape == (3, 3))
    
        F = np.eye(3)
        x_pred = np.dot(F, x)
        P_pred = np.dot(np.dot(F, P), F.T) + Q
        H = np.eye(3)
        y_residual = z - np.dot(H, x_pred)
        S = np.dot(np.dot(H, P_pred), H.T) + R
        K_gain = np.dot(np.dot(P_pred, H.T), np.linalg.inv(S))
        x_updated = x_pred + np.dot(K_gain, y_residual)
        P_updated = P_pred - np.dot(np.dot(K_gain, H), P_pred)
        return x_updated, P_updated
    
    def get_linear_acceleration_earth(self):
        """
        Converts the accelerometer reading from the body frame to the Earth frame
        and subtracts gravity to yield the net (linear) acceleration.
    
        Returns:
            linear_acc (numpy array): The net acceleration vector in the Earth frame.
        """
        # Create a numpy array from body-frame acceleration values.
        acc_body = np.array([self.accel['x'], self.accel['y'], self.accel['z']])
        # Use the rotation matrix from the EKF if available (self.matrix); else, assume identity.
        if hasattr(self, 'matrix') and self.matrix is not None:
            acc_earth = np.dot(self.matrix, acc_body)
        else:
            acc_earth = acc_body.copy()
        # Gravity vector: assuming Down is positive in Earth frame.
        gravity = np.array([0, 0, 9.81])
        linear_acc = acc_earth - gravity
        return linear_acc
    
    def run(self):
        self.poll()
        return self.euler, self.accel, self.gyro
    
    def run_threaded(self):
        return self.euler, self.accel, self.gyro
    
    def shutdown(self):
        if self.ser:
            self.ser.close()
            logger.info("Closed serial connection")

# ---------- Free Functions ---------- #

def log_data(artemis_imu: ArtemisOpenLog):
    """
    Plots the data in real time.

    :param: artemis_imu
        The ArtemisOpenLog object to use to collect the data.
    """
    try:
        def update_plot(_):
            artemis_imu.poll()
            # line_gx.set_data(range(len(artemis_imu.gyro_x)), list(artemis_imu.gyro_x))
            # line_gy.set_data(range(len(artemis_imu.gyro_y)), list(artemis_imu.gyro_y))
            line_gz.set_data(range(len(artemis_imu.gyro_z)), list(artemis_imu.gyro_z))

            # line_gx_s.set_data(range(len(artemis_imu.gyro_x_s)), list(artemis_imu.gyro_x_s))
            # line_gy_s.set_data(range(len(artemis_imu.gyro_y_s)), list(artemis_imu.gyro_y_s))
            line_gz_s.set_data(range(len(artemis_imu.gyro_z_s)), list(artemis_imu.gyro_z_s))

            ax.set_xlim(0, len(artemis_imu.gyro_x))
            # return line_gx, line_gy, line_gz, line_gx_s, line_gy_s, line_gz_s
            return line_gz, line_gz_s

        fig, ax = plt.subplots()
        ax.set_xlabel("Time")
        ax.set_ylabel("Gyroscope (deg/s)")
        ax.set_ylim(-10, 10)
        # line_gx, = ax.plot([], [], label="gX", color="r")
        # line_gy, = ax.plot([], [], label="gY", color="g")
        line_gz, = ax.plot([], [], label="gZ", color="b")

        # line_gx_s, = ax.plot([], [], label="gX_s", color="orange")
        # line_gy_s, = ax.plot([], [], label="gY_s", color="purple")
        line_gz_s, = ax.plot([], [], label="gZ_s", color="red")        
        ax.legend()

        ani = animation.FuncAnimation(fig, update_plot, interval=5, blit=True, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        logger.info("\nStopping the plotting...")

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

