#!/usr/bin/env python3
import math
import time
import board
import busio
import adafruit_mpu6050
import logging
import imufusion

import numpy as np
import pandas as pd
from tensorflow.python.ops.metrics_impl import precision

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
    def __init__(self):
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
        self.time = None
        self.path = [] # [(self.time, *self.pos)]
        self.sample_rate = 100
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
        self.calibrate()

    def calibrate(self):
        logger.info('Calibrating Mpu6050 ...')
        num_loops = 100
        gyro = np.zeros(3)
        accel = np.zeros(3)
        for _ in range(num_loops):
            gyro += self.mpu.gyro
            accel += self.mpu.acceleration
            time.sleep(0.01)
        self.gyro_zero = gyro / num_loops
        self.accel_zero = accel / num_loops
        while self.ahrs.flags.initialising:
            self.poll()
            time.sleep(0.01)
        self.time = time.time()
        logger.info('Mpu6050 calibrated')

    def update(self):
        while self.on:
            self.poll()

    def poll(self):
        new_time = time.time()
        if self.time is None:
            self.time = new_time
        dt = new_time - self.time
        gyro = np.array(self.mpu.gyro) - self.gyro_zero
        # convert from radians to degrees
        gyro_degree = np.array(gyro) * 180 / math.pi
        adj_gyro = self.offset.update(gyro_degree)
        accel_phys = np.array(self.mpu.acceleration)
        self.ahrs.update_no_magnetometer(adj_gyro, accel_phys/9.81, dt)
        if not self.ahrs.flags.initialising:
            self.euler = self.ahrs.quaternion.to_euler()
            self.matrix = self.ahrs.quaternion.to_matrix()
            self.lin_accel = accel_phys - self.accel_zero
            delta_v = np.dot(self.matrix, self.lin_accel) * dt
            self.speed += delta_v
            self.pos += self.speed * dt
            self.path.append((self.time, *self.pos))
        self.time = new_time

    def run(self):
        self.poll()
        return self.euler, self.matrix, self.lin_accel

    def run_threaded(self):
        return self.accel, self.gyro

    def shutdown(self):
        self.on = False
        df = pd.DataFrame(columns=['t', 'x', 'y', 'z',], data=self.path)
        df.to_csv('imu.csv', index=False)


if __name__ == "__main__":
    import sys
    from sys import stdout
    np.set_printoptions(precision=3, sign='+', floatmode='fixed', suppress=True)
    count = 0
    p = Mpu6050Ada()
    print('\n', '\n')
    while True:
        try:
            euler, matrix, accel = p.run()
            #out_str = f"\reuler: " + f",".join(f"{x:+5.3f}" for x in matrix)
            out_str = f"\rm = "
                # np.array2string(matrix, precision=3, separator=',',
                #                 sign='+', floatmode='fixed',
                #                 suppress_small=True).replace('\n', '') +\
            out_str += \
                'a = ' + \
                np.array2string(accel, precision=3, separator=',', sign='+',
                                floatmode='fixed',
                                suppress_small=True).replace('\n', '')
            stdout.write(out_str)
            stdout.flush()
            time.sleep(0.01)
            count += 1
        except KeyboardInterrupt:
            p.shutdown()
            stdout.write("\n")
            break
    sys.exit(0)

    iter = 0
    import sys
    sensor_type = SENSOR_MPU6050
    dlp_setting = DLP_SETTING_DISABLED
    if len(sys.argv) > 1:
        sensor_type = sys.argv[1]
    if len(sys.argv) > 2:
        dlp_setting = int(sys.argv[2])

    p = IMU(sensor=sensor_type)
    while iter < 100:
        data = p.run()
        print(data)
        time.sleep(0.1)
        iter += 1

