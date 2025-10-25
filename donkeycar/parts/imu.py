#!/usr/bin/env python3
import numpy as np
import math
import time
import os
import board
import busio
import adafruit_mpu6050
import adafruit_bno055
import logging

import pandas as pd


logger = logging.getLogger(__name__)

SENSOR_MPU6050 = 'mpu6050'
SENSOR_MPU9250 = 'mpu9250'

DLP_SETTING_DISABLED = 0
CONFIG_REGISTER = 0x1A


def _save_imu_path_to_csv(path_data, filepath, class_name):
    """
    Save IMU path data to CSV and verify file exists.

    Args:
        path_data: List of path tuples (t, x, y, z, v)
        filepath: Target CSV file path
        class_name: Name of calling class for logging
    """
    df = pd.DataFrame(columns=['t', 'x', 'y', 'z', 'v'], data=path_data)
    df.to_csv(filepath, index=False)

    for attempt in range(10):
        if os.path.exists(filepath):
            logger.info(
                f'{class_name} - saved {filepath}, {attempt+1} attempts')
            return
        time.sleep(0.1)
        df.to_csv(filepath, index=False)

    logger.error(f'{class_name} - {filepath} not found after write')


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

    def __init__(self, addr=0x68, poll_delay=0.0166,
                 sensor=SENSOR_MPU6050, dlp_setting=DLP_SETTING_DISABLED):
        self.sensortype = sensor
        if self.sensortype == SENSOR_MPU6050:
            from mpu6050 import mpu6050 as MPU6050
            self.sensor = MPU6050(addr)

            if (dlp_setting > 0):
                self.sensor.bus.write_byte_data(self.sensor.address,
                                                CONFIG_REGISTER, dlp_setting)

        else:
            from mpu9250_jmdev.registers import (AK8963_ADDRESS, GFS_1000,
                                                 AFS_4G, AK8963_BIT_16, AK8963_MODE_C100HZ)
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

            if (dlp_setting > 0):
                self.sensor.writeSlave(CONFIG_REGISTER, dlp_setting)
            self.sensor.calibrateMPU6500()
            self.sensor.configure()

        self.accel = {'x': 0., 'y': 0., 'z': 0.}
        self.gyro = {'x': 0., 'y': 0., 'z': 0.}
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
                self.accel = {
                    'x': ret[1] * GRAVITY,
                    'y': ret[2] * GRAVITY,
                    'z': ret[3] * GRAVITY}
                self.gyro = {'x': ret[4], 'y': ret[5], 'z': ret[6]}
                self.mag = {'x': ret[13], 'y': ret[14], 'z': ret[15]}
                self.temp = ret[16]
        except BaseException:
            print('failed to read imu!!')

    def run_threaded(self):
        return \
            [a - z for a, z in zip(list(self.accel.values()), self.accel_zero)], \
            [g - z for g, z in zip(list(self.gyro.values()), self.gyro_zero)]

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.on = False


class Mpu6050Ada:
    def __init__(self, sample_rate=100):
        import imufusion
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
        self.path = []  # [(self.time, *self.pos)]
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
        for _ in range(num_loops // 2):
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
        self.accel = ((1 - alpha) * self.accel
                      + alpha * np.array(self.mpu.acceleration))
        self.ahrs.update_no_magnetometer(
            adj_gyro, self.accel / self.accel_norm, dt)
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
        _save_imu_path_to_csv(self.path, 'imu.csv', 'Mpu6050')


class BNO055Ada:
    def __init__(self, alpha=1.0, record_path=False):
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.last_val = 0xFFFF
        self.sensor.offsets_accelerometer = (41, -116, -26)
        self.sensor.offsets_gyroscope = (-1, -1, -1)
        self.sensor.offsets_magnetometer = (-176, 196, 17)
        self.pos = np.zeros(3)  # [x, y, z] where x=forward, y=left, z=up
        self.accel = np.zeros(3)  # np.array(self.sensor.linear_acceleration)
        self.gyro = np.array(self.sensor.gyro)
        self.path = []
        self.time = None
        self.on = True
        # euler angles are in z, y, x order in the sensor
        self.euler = np.array(self.sensor.euler[::-1])
        self.alpha = alpha
        self.record_path = record_path
        self.odometer_speed = 0.0
        logger.info(f"Created BNO055, with alpha={self.alpha}, "
                    f"record_path={self.record_path}")

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
        self.gyro *= (1.0 - self.alpha)
        self.gyro += self.alpha * np.array(self.sensor.gyro)

        self.accel *= (1.0 - self.alpha)
        self.accel += self.alpha * np.array(self.sensor.linear_acceleration)

        # euler angles are in z, y, x order in the sensor
        euler_reading = np.array(self.sensor.euler[::-1])
        # Ignore readings when BNO055 returns [0, 0, 0] (sensor error)
        logger.debug(f'BNO055 euler: {euler_reading}, gyro: {self.gyro}, '
                     f'speed: {self.odometer_speed}')
        if np.any(euler_reading):
            self.euler *= (1.0 - self.alpha)
            self.euler += self.alpha * euler_reading

        if self.record_path and self.odometer_speed is not None:
            # Use 2D position estimation with odometer speed and heading (Euler z-angle)
            # Euler z-angle (heading/yaw) is in degrees, 360° = 0° = forward (parallel to y-axis)
            # Convert to standard math coordinates (0° = +y, 90° = +x)
            # add angular correction
            heading_deg = (90.0 - self.euler[2]) + 1.0
            heading_rad = math.radians(heading_deg)

            # Calculate 2D velocity components using odometer speed and heading
            # X-axis: forward direction, Y-axis: left direction
            vx = self.odometer_speed * math.cos(heading_rad)  # forward velocity
            vy = self.odometer_speed * math.sin(heading_rad)  # left velocity

            # Update 2D position using kinematics
            self.pos[0] += vx * dt  # x position (forward)
            self.pos[1] += vy * dt  # y position (left)
            self.pos[2] = 0.0       # z position always zero (2D plane)

            # correction term
            # step_distance = self.odometer_speed * dt
            # self.pos[0] -= step_distance * 0.01

            # Store path with 2D coordinates and odometer speed
            self.path.append((self.time, self.pos[0], self.pos[1],
                              0.0, self.odometer_speed))

        self.time = new_time

    def update(self):
        while self.on:
            self.poll()

    def run_threaded(self, odometer_speed=None):
        if odometer_speed is not None:
            self.odometer_speed = odometer_speed
        return self.euler, self.accel, self.gyro

    def run(self, odometer_speed=None):
        if odometer_speed is not None:
            self.odometer_speed = odometer_speed
        self.poll()
        return self.euler, self.accel

    def shutdown(self):
        self.on = False
        logger.info("Shutting down BNO055...")
        if not self.record_path:
            return
        _save_imu_path_to_csv(self.path, 'imu.csv', 'BNO055')


if __name__ == "__main__":
    import sys
    from sys import stdout
    import threading
    from donkeycar.utilities.imu_path_visualizer import PathPlotter

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
            # out_str = f"\reuler: " + f",".join(f"{x:+5.3f}" for x in matrix)
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
