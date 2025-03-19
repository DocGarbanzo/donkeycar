import numpy as np
from scipy.spatial.transform import Rotation as R

class EKF_IMU:
    def __init__(self, dt):
        self.dt = dt  
        # State vector: quaternion [qw, qx, qy, qz]
        self.x = np.array([1.0, 0.0, 0.0, 0.0]) 
        self.P = np.eye(4) * 0.01  
        self.Q = np.eye(4) * 1e-5  
        self.R_acc = np.eye(3) * 0.01  
        self.R_mag = np.eye(3) * 0.01  

    def predict(self, gyro):
        """
        Predict step using gyroscope data

        :param: gyro
            Numpy array of gyroscope data
        """
        wx, wy, wz = gyro  
        omega = np.array([
            [ 0, -wx, -wy, -wz],
            [ wx,  0,  wz, -wy],
            [ wy, -wz,  0,  wx],
            [ wz,  wy, -wx,  0]
        ])
        dq = 0.5 * omega @ self.x * self.dt
        self.x += dq
        self.x /= np.linalg.norm(self.x)
        self.P += self.Q  

    def update(self, acc, mag):
        """
        Update step using accelerometer and magnetometer data
        
        :param: acc
            Numpy array of acceleration data
        :param: mag
            Numpy array of magnetometer data
        """
        acc = acc / np.linalg.norm(acc)  
        mag = mag / np.linalg.norm(mag)  

        # Expected gravity vector in world frame
        g_ref = np.array([0, 0, 1])
        g_meas = self.quat_rotate(self.x, g_ref)
        acc_error = acc - g_meas
        m_ref = np.array([1, 0, 0])
        m_meas = self.quat_rotate(self.x, m_ref)
        mag_error = mag - m_meas
        y = np.hstack((acc_error, mag_error))
        H = np.zeros((6, 4)) 
        H[:3, :3] = np.eye(3)
        H[3:, :3] = np.eye(3)

        R = np.block([[self.R_acc, np.zeros((3,3))], [np.zeros((3,3)), self.R_mag]])  # (6x6)
        S = H @ self.P @ H.T + R  # (6x6)
        K = self.P @ H.T @ np.linalg.inv(S)  # (4x6)

        # Update state
        self.x += K @ y
        self.x /= np.linalg.norm(self.x)
        # Update covariance
        self.P = (np.eye(4) - K @ H) @ self.P


    def quat_rotate(self, q, v):
        """
        Rotates vector v using quaternion q
        """
        r = R.from_quat(q)
        return r.apply(v)

    def get_euler_angles(self):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        """
        r = R.from_quat(self.x)
        return r.as_euler('xyz', degrees=True)  # Roll, Pitch, Yaw

if __name__ == "__main__":
    ekf = EKF_IMU(dt=0.01)

    gyro = np.array([0.01, -0.02, 0.005])  # Gyroscope (rad/s)
    acc = np.array([0.0, 0.0, 9.81])       # Accelerometer (m/s^2)
    mag = np.array([0.5, 0.1, 0.3])        # Magnetometer (arbitrary units)

    ekf.predict(gyro)
    ekf.update(acc, mag) 
    euler_angles = ekf.get_euler_angles()

    print(f"Roll: {euler_angles[0]:.2f}°, Pitch: {euler_angles[1]:.2f}°, Yaw: {euler_angles[2]:.2f}°")
