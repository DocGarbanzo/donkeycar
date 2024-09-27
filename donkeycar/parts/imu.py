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
        self.mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS
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
        self.calibrate()

    def calibrate(self):
        logger.info("Calibrating IMU ...")
        while self.ahrs.flags.initialising:
            self.poll()
            time.sleep(0.01)
        logger.info("Calibration done.")
        stdout.flush()

    def update(self):
        while self.on:
            self.poll()

    def poll(self):
        new_time = time.time()
        # if self.time is None:
        #     self.time = new_time
        # delta_t = new_time - self.time
        # # convert from radians to degrees
        # scaled_gyro = np.array(self.mpu.gyro) * 180 / math.pi
        # gyro = self.offset.update(scaled_gyro)
        # accel = (np.array(self.mpu.acceleration)
        #          / np.linalg.norm(self.mpu.acceleration)) # 9.81
        # self.ahrs.update_no_magnetometer(gyro, accel, delta_t)
        #
        # self.euler = self.ahrs.quaternion.to_euler()
        # self.matrix = self.ahrs.quaternion.to_matrix()
        #
        # # delta_v = np.dot(self.frame, self.accel) * delta_t
        # # self.speed += delta_v
        # # self.pos += self.speed * delta_t
        # # self.path.append((self.time, *self.pos))
        # self.time = new_time
        # if self.ahrs.flags.initialising:
        #     return
        # only record and calculate if not initialising
        self.path.append((new_time,
                          *self.mpu.gyro,
                          *self.mpu.acceleration))

    def run(self):
        self.poll()
        return self.euler

    def run_threaded(self):
        return self.accel, self.gyro

    def shutdown(self):
        self.on = False
        df = pd.DataFrame(columns=['t', 'gx', 'gy', 'gz', 'ax', 'ay', 'az'],
                          data=self.path)
        df.to_csv('imu.csv', index=False)


if __name__ == "__main__":
    import sys
    from sys import stdout
    count = 0
    p = Mpu6050Ada()
    while True:
        try:
            matrix = p.run()
            out_str = f"\reuler: " + f",".join(f"{x:+5.3f}" for x in matrix)
            #out_str = f"\rm = {matrix}"
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

