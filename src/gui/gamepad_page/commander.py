#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from rov_viewer.msg import SetVelocity 
from rov_viewer.msg import SetHeading 
from rov_viewer.msg import SetDepth
from time import sleep

class Commander():
    """Class Commander: Publish PWM to RC channels wich send these pwm to thrusters
    
    Allow to choose to publish pwm from controller script or from manual command with gamepad
    ROS topics subscribed:
    ----------------------
    '/Command/depth': pwm sent by depth controller    
    '/Command/heading': pwm sent by heading controller 
    '/Command/velocity': pwm sent by velocity controller
    '/Command/joy': pwm and buttons pressed from gamepad 
    '/Settings/set_depth': to allow or disallow pwm from depth controller to be publish on RC channels
    '/Settings/set_heading': to allow or disallow pwm from heading controller to be publish on RC channels
    '/Settings/set_velocity': to allow or disallow pwm from velocity controller to be publish on RC channels
    ROS topics published:
    ---------------------
    '/BlueRov2/rc_channel3/set_pwm': to publish pwm for ROV thrusters,  THROTTLE     
    '/BlueRov2/rc_channel4/set_pwm': to publish pwm for ROV thrusters,  YAW          
    '/BlueRov2/rc_channel5/set_pwm': to publish pwm for ROV thrusters,  FORWARD      
    '/BlueRov2/rc_channel6/set_pwm': to publish pwm for ROV thrusters,  LATERAL      
    '/BlueRov2/rc_channel8/set_pwm': to publish pwm for ROV thrusters,  CAMERA TILT  
    '/BlueRov2/arm': to arm or disarm the BlueRov2
    
    Attributes:
    -----------
    rate: rosrate
    pwm_neutral: int, 1500 for BlueRov2 thrusters
    pwm_depth: int, from '/Command/depth' published by depth_controller.py
    pwm_heading: int, from '/Command/heading' published by heading_controller.py
    pwm_velocity: int, from '/Command/velocity' published by velocity_controller.py
    arm: bool, True=>armed, False=>disarmed
    enable_depth_ctrl: bool, to allow commander to publish pwm from depth controller, read from\
            '/Settings/set_depth'
    enable_heading_ctrl: bool, to allow commander to publish pwm from heading controller, read from\
            '/Settings/set_heading'
    enable_velocity_ctrl: bool, to allow commander to publish pwm from velocity controller, read\
            from '/Settings/set_velocity'
    gamepad_axes: list, len=4, [THROTTLE,YAW,FORWARD, LATERAL], from '/Command/joy'
    gamepad_buttons: list, len=5, [ARM, OVERRIDE_CONTROLLER, PWM_CAM, LIGHT_DEC, LIGHT_INC, GAIN_LIGHT],\
            from '/Command/joy'
    override_controller: int, 1=MANUAL MODE with gamepad, 0=AUTOMATIC MODE from \
            controllers if there are enables. Define by the gamepad
    """

    def __init__(self, arm=True, pwm_velocity=1500, pwm_neutral=1500, rosrate=4):
        self.pub_rc3 = rospy.Publisher('/BlueRov2/rc_channel3/set_pwm', UInt16, queue_size=10) #THROTTLE
        self.pub_rc4 = rospy.Publisher('/BlueRov2/rc_channel4/set_pwm', UInt16, queue_size=10) #YAW
        self.pub_rc5 = rospy.Publisher('/BlueRov2/rc_channel5/set_pwm', UInt16, queue_size=10) #FORWARD
        self.pub_rc6 = rospy.Publisher('/BlueRov2/rc_channel6/set_pwm', UInt16, queue_size=10) #LATERAL
        self.pub_rc8 = rospy.Publisher('/BlueRov2/rc_channel8/set_pwm', UInt16, queue_size=10) #CAMERA TILT 
        #self.pub_rc9 = rospy.Publisher('/BlueRov2/rc_channel9/set_pwm', UInt16, queue_size=10) #LIGHTS 1
        #self.pub_rc10 = rospy.Publisher('/BlueRov2/rc_channel10/set_pwm', UInt16, queue_size=10) #LIGHTS 2
        self.pub_manual_control = rospy.Publisher('/BlueRov2/manual_control', Joy, queue_size=10)

        rospy.Subscriber('/BlueRov2/Command/depth', UInt16, self._callback_depth)
        rospy.Subscriber('/BlueRov2/Command/heading', UInt16, self._callback_heading)
        rospy.Subscriber('/BlueRov2/Command/velocity', UInt16, self._callback_velocity)
        rospy.Subscriber('/BlueRov2/Command/joy', Joy, self._callback_joy)

        rospy.Subscriber('/BlueRov2/Setting/set_depth', SetDepth, self._settings_depth_ctrl_callback)
        rospy.Subscriber('/BlueRov2/Setting/set_heading', SetHeading, self._settings_heading_ctrl_callback)
        rospy.Subscriber('/BlueRov2/Setting/set_velocity', SetVelocity, self._settings_velocity_ctrl_callback)

        #self.pub_arm = rospy.Publisher('/BlueRov2/Setting/arm', Bool, queue_size=10)
        self.rate = rospy.Rate(rosrate)
        
        self.pwm_velocity = pwm_velocity
        self.pwm_neutral = 1500
        self.armed = arm
        self.pwm_depth = 0
        self.pwm_heading = 0
        
        self.enable_depth_ctrl = False
        self.enable_heading_ctrl = False
        self.enable_velocity_ctrl = False

        self.override_controller = 0 # 0:automatic control, 1:gamepad control
        self.gamepad_axes = [self.pwm_neutral, self.pwm_neutral, self.pwm_neutral, self.pwm_neutral] # THROTTLE,YAW,FORWARD, LATERAL
        self.gamepad_buttons = [0,0,1500,0,0, 1100] # ARM, OVERRIDE_CONTROLLER, PWM_CAM, LIGHT_DEC, LIGHT_INC, PWM_LIGHT

    def _callback_depth(self,msg):
        """Read from '/Command/depth'
        
        ROS message:
        ------------
        UInt16
        """
        self.pwm_depth = msg.data
        print("pwm_depth: ", self.pwm_depth)

    def _callback_heading(self,msg):
        """Read from '/Command/heading'
        
        ROS message:
        ------------
        UInt16
        """
        self.pwm_heading = msg.data

    def _callback_velocity(self,msg):
        """Read from '/Command/velocity'
        
        ROS message:
        ------------
        UInt16
        """
        self.pwm_velocity = msg.data

    def _callback_joy(self,msg):
        """Read from '/Settings/joy'
        Change the value of override_controller
        ROS message:
        ------------
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        float32[] axes
        int32[] buttons
        """
        self.gamepad_axes = msg.axes
        self.gamepad_buttons = msg.buttons
        self.override_controller = self.gamepad_buttons[1]

    def _settings_depth_ctrl_callback(self,msg):
        """Read data from '/Settings/set_depth'
        ROS message:
        ------------
        bool enable_depth_ctrl
        uint16 pwm_max 
        uint32 KI
        uint32 KP
        uint32 KD
        """
        self.enable_depth_ctrl= msg.enable_depth_ctrl

    def _settings_heading_ctrl_callback(self,msg):
        """Read data from '/Settings/set_heading'
        ROS message:
        -------------
        bool enable_heading_ctrl
        uint16 pwm_max
        uint32 KP
        uint32 KD
        """
        self.enable_heading_ctrl= msg.enable_heading_ctrl

    def _settings_velocity_ctrl_callback(self,msg):
        """Read from '/Settings/velocity'
        ROS message:
        ------------
        bool enable_velocity_ctrl
        uint16 pwm_max 
        uint32 KP
        uint32 KD
        """
        self.enable_velocity_ctrl = msg.enable_velocity_ctrl
 
    def publish_controller_command(self):
        """In automatic mode : arm the BlueRov2 and publish only pwm from enable controller"""   
        # self.pub_arm.publish(self.armed)
        if self.enable_depth_ctrl:
            print('DEPTH CONTROLLER ENABLE')
            self.pub_rc3.publish(self.pwm_depth)
        if self.enable_heading_ctrl:
            print('HEADING CONTROLLER ENABLE')
            self.pub_rc4.publish(self.pwm_heading)
        if self.enable_velocity_ctrl:
            print('VELOCITY CONTROLLER ENABLE')
            self.pub_rc5.publish(self.pwm_velocity)

    def publish_gamepad_command(self):
        """In manual mode : deal with the gamepad inputs
        gamepad_axes = [THROTTLE, YAW, FORWARD, LATERAL]
        gamepad_buttons = [ARM, OVERRIDE_CONTROLLER, PWM_CAM, LIGHT_DEC, LIGHT_INC, GAIN_LIGHT]
        lights cannot be controlled yet
        """
        self.pub_arm.publish(self.gamepad_buttons[0])
        self.pub_rc8.publish(self.gamepad_buttons[2]) #CAMERA
        #self.pub_rc9.publish(self.gamepad_buttons[5])
        #self.pub_rc10.publish(self.gamepad_buttons[5])
        
        self.pub_rc3.publish(self.gamepad_axes[0]) # THROTTLE
        self.pub_rc4.publish(self.gamepad_axes[1]) # YAW
        self.pub_rc5.publish(self.gamepad_axes[2]) # FORWARD
        self.pub_rc6.publish(self.gamepad_axes[3]) # LATERAL
        
        msg = Joy()
        msg.buttons = [0]*16
        msg.buttons[14]=self.gamepad_buttons[4] # light increase gain
        msg.buttons[13]=self.gamepad_buttons[3] # light decrease gain
        self.pub_manual_control.publish(msg)

    def master_control(self):
        """Master control: 
        excecute publish_gamepad_command(Manual mode) or publish_controller_command(Automatic mode) according to override_controller parameter
        """
        if self.override_controller == 1:
            self.publish_gamepad_command()
        else :
            self.publish_controller_command()


if __name__ == "__main__":
    rospy.init_node('Commander', anonymous=True)
    cmd = Commander()
    while not rospy.is_shutdown():
        cmd.master_control()
        cmd.rate.sleep()