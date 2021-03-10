#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from bluerov_ros_playground.msg import Attitude
from bluerov_ros_playground.msg import Set_heading
from bluerov_ros_playground.msg import Set_target

PI = np.pi

class Heading_Controller():
    """ Class Heading_Controller: follow a yaw input in rad with a PD controller

    ROS topics subscribed:
    -----------------------
    '/BlueRov2/imu/atitude': to get yaw in [-pi,pi]rad from PX4 internal IMU
    '/Settings/set_heading': to get pwm_max, KP, KD for saturation and PD coefficients
    '/Settings/set_target': to get the yaw to follow
    
    ROS topics published:
    ---------------------
    '/Command/heading': publish pwm to send for thrusters, sent to ROV by commander.py 

    Attributes:
    -----------
    pwm_max: int in [1500,1900], 1500 = neutral, 1900 = maximal pwm for BlueRov2 thrusters \
            (T200 : https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/) 
    pwm_neutral: int 1500,  for BlueRov2 thrusters
    KP: int, proportional coefficient
    KD: int, derivative coefficient
    rate: rosrate 
    heading_desired: float in [-pi,pi], read from '/Settings/set_target'
    attitude: list, len=6, [ROLL, PITCH, YAW, ROLLSPEED, PITCHSPEED, YAWSPEED] read from \
            '/BlueRov2/imu/attitude'
    """

    def __init__(self, heading_desired=0, KP=35, KD=25, pwm_max=1500, pwm_neutral=1500,rosrate=4):
        self.pub_pwm = rospy.Publisher('/Command/heading', UInt16, queue_size=10)

        rospy.Subscriber('/BlueRov2/imu/attitude', Attitude, self._callback_att)
        rospy.Subscriber('/Settings/set_heading', Set_heading, self._callback_set_heading)
        rospy.Subscriber('/Settings/set_target', Set_target, self._callback_set_target)

        self.rate = rospy.Rate(rosrate)
        self.attitude = [0, 0, 0, 0, 0, 0] #[ROLL, PITCH, YAW, ROLLSPEED, PITCHSPEED, YAWSPEED]
        self.pwm_max = pwm_max
        self.pwm_neutral = pwm_neutral
        self.heading_desired = heading_desired
        self.KP = KP
        self.KD = KD
        
    def sawtooth (self, x):
        """Deal with 2*PI modulo
        
        Input:
        ------
        x: rad 
        """
        return (x+PI)%(2*PI)-PI         

    def _callback_att(self, msg):
        """Read data from '/BlueRov2/imu/attitude'

        ROS message :
        -------------
        Header header
        uint32 time_boot_ms
        float64 roll
        float64 pitch
        float64 yaw
        float64 rollspeed
        float64 pitchspeed
        float64 yawspeed
        """
        self.attitude = [msg.roll,
                         msg.pitch,
                         msg.yaw,
                         msg.rollspeed,
                         msg.pitchspeed,
                         msg.yawspeed]
                   
    def _callback_set_heading(self, msg):
        """Read data from '/Settings/set_heading'

        ROS message:
        -------------
        bool enable_heading_ctrl
        uint16 pwm_max
        uint32 KP
        uint32 KD
        """
        if msg.pwm_max < 1500:
            self.pwm_max = 1500
        else:
            self.pwm_max = msg.pwm_max
        self.KP = msg.KP 
        self.KD = msg.KD 

    def _callback_set_target(self, msg):
        """Read data from '/Settings/set_target'

        ROS message:
        -------------
        float64 depth_desired
        float64 heading_desired
        float64 velocity_desired
        """
        self.heading_desired = self.deg2rad(msg.heading_desired)

    def deg2rad(self,deg):
        """Convert [0-360]deg to [-pi,pi]rad => [180,360]deg ~ [-pi,0]rad  ,  [0,180]deg ~ [0,pi]rad
       
        Input:
        ------
        deg: int
        """
        if deg in range(0,181):
            return (deg * PI) / 180 
        if deg in range(181,361):
            return ((deg - 360) * PI) / 180

    def control(self, yaw, yawspeed):
        """ PD controller,  
        
        Input:
        ------
        yaw
        yawspeed

        Return:
        -------
        command calculated to follow the heading desired
        """ 

        return self.KP*self.sawtooth(yaw-self.heading_desired) + self.KD*yawspeed
    
    def saturation(self, pwm):
        """Saturate command
        
        Input:
        ------
        pwm: pwm from self.control 
        
        Return:
        -------
        pwm: int, published on '/Command/heading'
        """
        pwm_min = self.pwm_neutral - (self.pwm_max - self.pwm_neutral)
        if pwm > self.pwm_max :
            pwm = self.pwm_max
        if pwm < pwm_min:
            pwm = pwm_min
        return int(pwm)

    def main(self):
        yaw = self.attitude[2]
        yawspeed = self.attitude[5]
        u = self.control(yaw, yawspeed)
        pwm = self.pwm_neutral - u
        pwm = self.saturation(pwm)
        print("HEADING_DESIRED : {}, YAW_MESURED : {}, PWM : {}".format(self.heading_desired, yaw, pwm))
        #pub_rc4.publish(pwm)
        self.pub_pwm.publish(pwm)


if __name__ == "__main__":
    rospy.init_node('heading_controller', anonymous=True)
    heading_controller = Heading_Controller()
    while not rospy.is_shutdown():
        heading_controller.main()
        heading_controller.rate.sleep()
