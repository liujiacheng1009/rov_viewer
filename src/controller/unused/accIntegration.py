import numpy as np
import matplotlib.pyplot as plt
import time
from scipy import integrate
import rospy
from sensor_msgs.msg import Imu

class Integration():
    def __init__(self):
        rospy.Subscriber('/BlueRov2/imu/data', Imu, self._callback_imu_data)
        rospy.Subscriber('/BlueRov2/imu/imu1', Imu, self._callback_imu_1)
        rospy.Subscriber('/BlueRov2/imu/imu2', Imu, self._callback_imu_2)
        self.rosrate = 100
        self.rate = rospy.Rate(self.rosrate)
        self.dt = 1/100.

        self.accX_IMU1_exp = 0
        self.accX_IMU2_exp = 0
        self.accX_IMU1_array = np.zeros((34, ))
        self.accX_IMU2_array = np.zeros((34, ))
        self.accX_IMU1_hp = 0
        self.accX_IMU2_hp = 0
        self.speedX_IMU1_euler = 0
        self.speedX_IMU1_euler = 0
        self.w = 0.1
        self.highpass = signal.butter(10,0.001,'high', analog=False, output='ba')  

    def _callback_imu_1(self, msg):
        #moy : sur 40s 0.41110448395365074
        #moy : sur 3600s 0.35930319258263654
        self.acc_imu1 = [msg.linear_acceleration.x-0.35930319258263654,
                                        msg.linear_acceleration.y,
                                        msg.linear_acceleration.z]

    def _callback_imu_2(self, msg):
        #moy : 0.27067840479669764
        #moy : sur 3600s  0.23206011098693963
        self.acc_imu2 = [msg.linear_acceleration.x-0.23206011098693963,
                                    msg.linear_acceleration.y,
                                    msg.linear_acceleration.z]

    def HPfilter(self, enable=True):
        self.accX_IMU1_exp = self.w * self.acc_imu1[0]+(1-self.w)*self.accX_IMU1_exp
        self.accX_IMU2_exp = self.w * self.acc_imu2[0]+(1-self.w)*self.accX_IMU2_exp
        if enable :
            n = len(self.accX_IMU1_array)
            for i in range(0,n-1):
                self.accX_IMU1_array[i] = self.accX_IMU1_array[i+1]
                self.accX_IMU2_array[i] = self.accX_IMU2_array[i+1]
            self.accX_IMU1_array[33] = self.accX_IMU1_exp
            self.accX_IMU2_array[33] = self.accX_IMU2_exp
            self.accX_IMU1_hp= signal.filtfilt(self.highpass[0], self.highpass[1], self.accX_IMU1_array)
            self.accX_IMU2_hp= signal.filtfilt(self.highpass[0], self.highpass[1], self.accX_IMU2_array)
        else:
            self.accX_mean_exp = (self.acc_imu1[0]+self.acc_imu2[0])/2

    def get_accX_IMU1(self, t, y):
        return self.accX_IMU1

    def get_accX_IMU2(self, t, y):
        return self.accX_IMU2
    
    def RK4_step(self):
        #TODO
        k1 = G(y,t)
        k2 = G(y+0.5*k1*dt, t+0.5*dt)

    def eulerStep(self):
        #self.speedX_IMU1_RK45 = integrate.RK45(self.get_accX_IMU1, 0., 0., 600)   
        #self.speedX_IMU2_RK45 = integrate.RK45(self.get_accX_IMU2, 0., 0., 600)   
        self.HPfilter(False)
        self.speedX_IMU1_euler = self.speedX_IMU1 + self.dt*self.accX_IMU1_exp
        self.speedX_IMU2_euler = self.speedX_IMU2 + self.dt*self.accX_IMU2_exp
        self.speedX_MEAN_euler = self.speedX_MEAN_euler + self.dt * ((self.accX_IMU1_exp+self.accX_IMU2_exp)/2)

if __name__ == "__main__":
    rospy.init_node('imu_kalman', anonymous=True)
    itgr = Integration()
    file_data = open("euler_imu_43.csv", "w")
    file_data.write("time, IMU1_accX, IMU1_accX_exp, IMU1_accX_hp, IMU1_speedX, IMU2_accX, IMU2_accX_exp, IMU2_accX_hp, IMU2_speedX, MEAN_speedX") 
    t0 = time.time()
    while time.time()-t0 < 40:
        itgr.eulerStep()
        file_data.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(time.time()-t0,
                                                                            itgr.acc_imu1[0],
                                                                            itgr.accX_IMU1_exp,
                                                                            itgr.accX_IMU1_hp,
                                                                            itgr.speedX_IMU1_euler,
                                                                            itgr.acc_imu2[0],
                                                                            itgr.accX_IMU2_exp,
                                                                            itgr.accX_IMU2_hp,
                                                                            itgr.speedX_MEAN_euler))
        print(time.time()-t0)
    file_data.close()

