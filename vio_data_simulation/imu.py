# 将以下的C++代码转换为python
"""
struct MotionData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_gyro;

    Eigen::Vector3d imu_gyro_bias;
    Eigen::Vector3d imu_acc_bias;

    Eigen::Vector3d imu_velocity;
};
"""

import numpy as np
import math
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import os

# MotionData
class MotionData:
    def __init__(self):
        self.timestamp = 0.0
        self.Rwb = np.eye(3)
        self.twb = np.zeros((3, 1))
        self.imu_acc = np.zeros((3, 1))
        self.imu_gyro = np.zeros((3, 1))
        self.imu_gyro_bias = np.zeros((3, 1))
        self.imu_acc_bias = np.zeros((3, 1))
        self.imu_velocity = np.zeros((3, 1))

def euler2Rotation(eulerAngles):
    roll = eulerAngles[0]
    pitch = eulerAngles[1]
    yaw = eulerAngles[2]

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    RIb = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                    [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                    [-sp  , cp*sr         , cp*cr          ]])
    return RIb

def eulerRates2bodyRates(eulerAngles):
    roll = eulerAngles[0]
    pitch = eulerAngles[1]

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)

    R = np.array([[1, 0, -sp],
                  [0, cr, sr*cp],
                  [0, -sr, cr*cp]])
    return R


class IMU:
    def __init__(self, param):
        self.param = param
        self.gyro_bias = np.zeros((3, 1))
        self.acc_bias = np.zeros((3, 1))
        self.init_velocity = np.zeros((3, 1))
        self.init_twb = np.zeros((3, 1))
        self.init_Rwb = np.eye(3)

    def addIMUnoise(self, data):
        noise_gyro = np.random.normal(0, 1, (3, 1))
        gyro_sqrt_cov = self.param.gyro_noise_sigma * np.eye(3)
        data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / math.sqrt( self.param.imu_timestep ) + self.gyro_bias

        noise_acc = np.random.normal(0, 1, (3, 1))
        acc_sqrt_cov = self.param.acc_noise_sigma * np.eye(3)
        data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / math.sqrt( self.param.imu_timestep ) + self.acc_bias

        # gyro_bias update
        noise_gyro_bias = np.random.normal(0, 1, (3, 1))
        self.gyro_bias += self.param.gyro_bias_sigma * math.sqrt(self.param.imu_timestep ) * noise_gyro_bias
        data.imu_gyro_bias = self.gyro_bias

        # acc_bias update
        noise_acc_bias = np.random.normal(0, 1, (3, 1))
        self.acc_bias += self.param.acc_bias_sigma * math.sqrt(self.param.imu_timestep ) * noise_acc_bias
        data.imu_acc_bias = self.acc_bias
  

    def MotionModel(self, t):
        data = MotionData()
        # param
        ellipse_x = 15
        ellipse_y = 20
        z = 1           # z轴做sin运动
        K1 = 10          # z轴的正弦频率是x，y的k1倍
        K = math.pi/ 10    # 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

        # translation
        # twb:  body frame in world frame
        
        position = np.array([ellipse_x * math.cos( K * t) + 5, ellipse_y * math.sin( K * t) + 5,  z * math.sin( K1 * K * t ) + 5])

        dp = np.zeros((3, 1))       # position导数　in world frame
        ddp = np.zeros((3, 1))      # position二阶导数

        # Rotation
        k_roll = 0.1
        k_pitch = 0.2
        eulerAngles = np.array([k_roll * math.sin(t) , k_pitch * math.sin(t) , K*t ])   # roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        eulerAnglesRates = np.zeros((3, 1))      # euler angles 的导数

        Rwb = euler2Rotation(eulerAngles)         # body frame to world frame
        imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates # euler rates trans to body gyro

        gn = np.array([0,0,-9.81])                                   #  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        imu_acc = Rwb.transpose() * ( ddp -  gn ) #  Rbw * Rwn * gn = gs

        data.imu_gyro = imu_gyro
        data.imu_acc = imu_acc
        data.Rwb = Rwb
        data.twb = position
        data.imu_velocity = dp
        data.timestamp = t
        return data
    
    # 读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
    # 用来验证数据以及模型的有效性。
    def testImu(self, src, dist):
        imudata = []
        self.LoadPose(src, imudata)

        save_points = open(dist, 'w')

        dt = self.param.imu_timestep
        Pwb = self.init_twb
        Qwb = self.init_Rwb
        Vw = self.init_velocity
        gw = np.array([0,0,-9.81])    # ENU frame
        for i in range(1, len(imudata)):
            imupose = imudata[i]

            # delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
            dq = np.zeros((4, 1))
            dtheta_half = imupose.imu_gyro * dt / 2.0
            dq[0] = 1
            dq[1:4] = dtheta_half
            dq = dq / np.linalg.norm(dq)
            # imu 动力学模型 参考svo预积分论文
            acc_w = Qwb.dot(imupose.imu_acc) + gw
            Qwb = Qwb.dot(dq)
            Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w
            Vw = Vw + acc_w * dt

            # 按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
            save_points.write(str(imupose.timestamp) + ' ' + str(Qwb[0]) + ' ' + str(Qwb[1]) + ' ' + str(Qwb[2]) + ' ' + str(Qwb[3]) + ' ' + str(Pwb[0]) + ' ' + str(Pwb[1]) + ' ' + str(Pwb[2]) + ' ' + str(Qwb[0]) + ' ' + str(Qwb[1]) + ' ' + str(Qwb[2]) + ' ' + str(Qwb[3]) + ' ' + str(Pwb[0]) + ' ' + str(Pwb[1]) + ' ' + str(Pwb[2]) + '\n')

        print("test end")