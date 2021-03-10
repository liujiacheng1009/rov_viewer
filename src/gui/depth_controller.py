#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from bluerov_ros_playground.msg import Bar30
from bluerov_ros_playground.msg import Set_depth
from bluerov_ros_playground.msg import Set_target
#axis z goes up

g = 9.81  # m.s^-2 gravitational acceleration 
p0 = 990*100 #Pa surface pressure NEED to be cheked
rho = 1000 # kg/m^3  water density

class Depth_Control():
    """ Class Depth_Control: reach depth desired with PID control
    
    WARNING : In this script, the axis Z for the depth goes up. 
              To reach 1m depth the input must be -1

    ROS topics subscribed:
    -----------------------
    '/BlueRov2/bar30': get absolute pressure in hPa
    '/Settings/set_depth': to get pwm_max, KP, KI, KD for saturation and PID coefficients
    '/Settings/set_target': to get depth to reach 
    
    ROS topics published:
    ---------------------
    '/Command/depth': publish pwm to send for thrusters, sent to ROV by commander.py 

    Attributes:
    -----------
    pwm_max: int in [1500,1900], 1500 = neutral, 1900 = maximal pwm for BlueRov2 thrusters \
            (T200 : https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/) 
    pwm_neutral: int 1500,  for BlueRov2 thrusters
    KP: int, proportional coefficient
    KI: int, integral coefficient
    KD: int, derivative coefficient
    rate: rosrate 
    depth_desired: float, in meter, read from '/Settings/set_target'
    bar30_data: list, len=4,  [time_boot_ms, press_abs, press_diff, temperature] read from \
            '/BlueRov2/bar30'
    depth: the altitude of ROV calculated with pressure measured by bar30 sensor
    time: read from '/BlueRov2/bar30', from sensor measure, for intefration
    I_depth: integral of depth
    """
    def __init__(self, depth_desired=0, pwm_max=1500, pwm_neutral=1500, rosrate=4):
        self.pub_pwm = rospy.Publisher('/Command/depth', UInt16, queue_size=10)
        rospy.Subscriber('/BlueRov2/bar30', Bar30, self._callback_bar30)
        rospy.Subscriber('/Settings/set_depth', Set_depth, self._callback_set_depth)
        rospy.Subscriber('/Settings/set_target', Set_target, self._callback_set_target)

        self.rate = rospy.Rate(rosrate)
        self.depth_desired = depth_desired
        self.bar30_data = [0, 0, 0, 0] # [time_boot_ms, press_abs, press_diff, temperature]
        self.pwm_max = pwm_max
        self.pwm_neutral = pwm_neutral
        self.KI = 100
        self.KP = 600
        self.KD = 50

        self.time = 0
        self.depth = 0
        self.I_depth = 0
        

    def _callback_bar30(self, msg):
        """Read data from '/BlueRov2/bar30'

        ROS message:
        Header header
        uint32 time_boot_ms
        float64 press_abs
        float64 press_diff
        int16 temperature
        """
        self.bar30_data = [ msg.time_boot_ms,
                            msg.press_abs,
                            msg.press_diff,
                            msg.temperature ]
        	                
    def _callback_set_depth(self, msg):
        """Read data from '/Settings/set_depth'

        ROS message:
        ------------
        bool enable_depth_ctrl
        uint16 pwm_max 
        uint32 KI
        uint32 KP
        uint32 KD
        """
        if msg.pwm_max < 1500:
            self.pwm_max = 1500
        else:
            self.pwm_max = msg.pwm_max
        self.KI = msg.KI 
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
        self.depth_desired = msg.depth_desired

    def control_pid(self, p):
        """PID controller
        Transform pressure to depth value
        Calulate the integrate value with euler method

        Input:
        ------
        p: absolute presssure in Pa

        Return:
        -------
        command calculated to reach the depth desired

        """
        depth  = -(p-p0)/(rho*g)
        delta_depth = depth - self.depth
        self.depth = depth #current depth
        delta_t = (self.bar30_data[0] - self.time)/1000.
        self.time = self.bar30_data[0]

        if delta_t == 0:
            D_depth = 0
        else:
            D_depth = delta_depth/delta_t #derivative term 

        self.I_depth = (self.depth_desired-depth)*delta_t #integrate term
        u = self.KI*self.I_depth + self.KP*(self.depth_desired-depth) - self.KD*D_depth
        return u
	
    def saturation(self, pwm):
        """Saturate command
        
        Input:
        ------
        pwm: pwm from self.control 
        
        Return:
        -------
        pwm: int, published on '/Command/depth'
        """
        pwm_min = self.pwm_neutral - (self.pwm_max - self.pwm_neutral)
        if pwm > self.pwm_max :
                pwm = self.pwm_max
        if pwm < pwm_min:
                pwm = pwm_min
        return int(pwm)

    def main(self):
        mesured_pressure = self.bar30_data[1]*100 #to convert pressure from hPa to Pa
        u = self.control_pid(mesured_pressure)
        pwm = 1500 + u
        pwm = self.saturation(pwm)
        print("DESIRED_DEPTH : {}, PRESSURE_MESURED : {}, PWM : {}".format(self.depth_desired, mesured_pressure, pwm))
        self.pub_pwm.publish(pwm)

if __name__ == "__main__":
    rospy.init_node('depth_controller', anonymous=True)
    depth_control = Depth_Control()
    
    while not rospy.is_shutdown():
        depth_control.main()
        depth_control.rate.sleep()
