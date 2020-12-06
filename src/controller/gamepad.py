#!/usr/bin/env python3

from __future__ import print_function
import rospy
from inputs import get_gamepad
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import sys

class Gamepad():
    """Class Gamepad: link between gamepad and ROS. Creating for Logitech Gamepad F310
    
    WARNING: light stuff doesn't work yet

    ROS topics suscribed:
    ---------------------
    '/BlueRov2/arm': to know at every time the satus of the BlueRov2

    ROS topics published:
    ---------------------
    '/Command/joy'

    Attributes:
    -----------
    rate: rosrate, unused
    pwm_max: int, to saturate gamepad command
    pwm_neutral: int, 1500 for BlueRov2 thrusters
    gain_pwm_cam: int, gain for camera tilt speed 
    list_buttons_clicked: list, len=16, 1=clicked, 0=not clicked, Not used yet but has the format \
            of manual_control for mavlink command in /bridge/bridge.py  
    override_controller: int, 1=Manual control, 0=Automatic control
    armed: bool, True=arm, False=disarm, read from '/BlueRov2/arm'/
    gain_light: int, store the gain for lights
    inc_gain_light: int, step for increasing or decreasing gain_light
    input: dict, link between button named from gamepad and method to call.
    """
    def __init__(self, pwm_max=1900, pwm_neutral=1500, gain_pwm_cam = 400, rosrate=4):
        self.pub = rospy.Publisher('/Command/joy', Joy, queue_size=10)
        self.sub = rospy.Subscriber('/BlueRov2/arm', Bool, self._arm_callback) 
        self.rate = rospy.Rate(rosrate)
        self.model_base_link = '/base_link'
        self.pwm_max = pwm_max
        self.pwm_neutral = pwm_neutral
        self.gain_pwm_cam = gain_pwm_cam  
        
        self.list_buttons_clicked = [0]*16
        self.override_controller = 1 # 1 to override all pwm sended by controllers
        self.armed = False # False if BlueRov2 disarmed, True if armed 
        #sensor_msgs/Joy :
        #  std_msgs/Header header
        #    uint32 seq
        #    time stamp
        #    string frame_id
        #  float32[] axes -> [THROTTLE, YAW, FORWARD, LATERAL]
        #  int32[] buttons ->[ARM, OVERRIDE_CONTROLLER, PWM_CAM, LIGHT_DEC, LIGHT_INC, GAIN_LIGHT]
        self.msg = Joy()
        self.msg.axes = [self.pwm_neutral, 
                self.pwm_neutral, 
                self.pwm_neutral,
                self.pwm_neutral]

        self.msg.buttons = [self.armed, 
                self.override_controller, 
                self.pwm_neutral,
                0,
                0,
                1100]
       
        self.gain_light = 1100
        self.inc_gain_light = 100

        #device : logitech gamepad F310
        self.input = {'ABS_Y': self._throttle, # LEFT stick vertical [0-255], 128 = neutral
            'ABS_X': self._lateral, # LEFT stick horizontal [0-255], 128 = neutral
            
            'ABS_RZ': self._forward, # RIGHT stick vertical [0-255], 128 = neutral
            'ABS_Z': self._yaw,  # RIGHT stick horizontal [0-255], 128 = neutral
            
            'ABS_HAT0Y': self._NDEF, # id ArduSub[up:11, down:12], LEFT cross vertical -1 left, up ; +1 right, down
            'ABS_HAT0X': self._set_gain_light, #id ArduSub [left:13, right:14], LEFT cross horizontal [-1;0;1]
           
            # id ArduSub : 7, LEFT stick click NONE DEF in INPUTS
            # id ArduSub : 8, RIGHT stick click NONE DEF in INPUTS

            
            'BTN_TRIGGER': self._override_controller, # id Ardusub : 2, X [0;1]
            'BTN_TOP1': self._NDEF, # id Ardusub : 3, Y [0;1]
            'BTN_THUMB': self._NDEF, # id Ardusub : 1, A [0;1]
            'BTN_THUMB2': self._NDEF, # id Ardusub : 0, B [0,1]

            'BTN_BASE': self._NDEF, # LT [0;1]
            'BTN_BASE2': self._NDEF, # RT [0;1]
            'BTN_TOP2': self._cam_down, # id Ardusub : 9, LB [0;1]
            'BTN_PINKIE': self._cam_up, # id Ardusub : 10,  RB [0;1]
            
            'BTN_BASE3': self._disarm, # id Ardusub : 4, BACK [0;1]
            'BTN_BASE4': self._arm, # id Ardusub : 6, START [0;1]
            }
        self.msg_header()
        self.pub.publish(self.msg)

    def _arm_callback(self, msg):
        """Read data from '/BlueRov2/arm'

        ROS message:
        ------------
        Bool data
        """
        self.armed = msg.data
        print('ARM ', self.armed)    

    def msg_header(self):
        "Create the header of the Joy message"
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.model_base_link

    def _NDEF(self, key, state):
        """Method called when not assigned button from gamepad is pressed see self.input dictionary"""
        print('{} not BIND, state : {}'.format(key, state))

    def _arm(self, key, state):
        if not self.armed :
            self.armed = True
        self.list_buttons_clicked[6] = state
        self.msg.buttons[0] = self.armed
        print("ARM, key : {}, state : {}, arm : {}".format(key, state, self.armed))

    def _disarm(self, key, state):
        if self.armed:
            self.armed = False 
        self.list_buttons_clicked[4] = state
        self.msg.buttons[0] = self.armed
        print("DISARM, key : {}, state : {}, arm : {}".format(key, state, self.armed))

    def _override_controller(self, key, state):
        if state == 1 and self.override_controller == 0:
            self.override_controller = 1
        elif state == 1 and self.override_controller == 1 :
            self.override_controller = 0
        self.msg.buttons[1] = self.override_controller
        print("OVERRIDE_CONTROLLER, key : {}, state, {}, override_controller : {}".format(key, state, self.override_controller))

    def _throttle(self, key, state):
        state = 255-state #to fix 255 top, 0 down (default : 0 up, 255 down)
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[0] = self.pwm_set_neutral(pwm)
        print("THROTTLE, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[0]))

    def _yaw(self, key,state):
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[1] = self.pwm_set_neutral(pwm)
        print("YAW, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[1]))

    def _forward(self, key, state):
        state = 255-state #to fix 255 top, 0 down (default : 0 up, 255 down)
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[2] = self.pwm_set_neutral(pwm)
        print("FORWARD, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[2]))

    def _lateral(self, key,state):
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[3] = self.pwm_set_neutral(pwm)
        print("LATERAL, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[3]))

    def _cam_up(self, key, state):
        if state == 1:
            pwm = self.pwm_neutral + self.gain_pwm_cam 
        else :
            pwm = self.pwm_neutral
        self.list_buttons_clicked[10] = state
        self.msg.buttons[2] = pwm
        print("CAM_UP, key : {}, state : {}, pwm : {}".format(key, state, self.msg.buttons[2]))

    def _cam_down(self, key, state):
        if state == 1:
            pwm = self.pwm_neutral - self.gain_pwm_cam 
        else :
            pwm = self.pwm_neutral
        self.list_buttons_clicked[9] = state
        self.msg.buttons[2] = pwm
        print("CAM_DOWN, key : {}, state : {}, pwm : {}".format(key, state, self.msg.buttons[2]))
    
    def _set_gain_light(self, key, state):
        if state == 1:
            self.msg.buttons[4] = 1
            self.list_buttons_clicked[14] = 1

            self.gain_light = self.gain_light+self.inc_gain_light
            self.msg.buttons[5] = self.gain_light

        elif state == -1:
            self.msg.buttons[3] = 1
            self.list_buttons_clicked[13]=1

            self.gain_light = self.gain_light-self.inc_gain_light
            self.msg.buttons[5] = self.gain_light

        else:
            self.msg.buttons[3] = 0
            self.msg.buttons[4] = 0
            self.list_buttons_clicked[13] = 0
            self.list_buttons_clicked[14] = 0 
        print("SET_GAIN_LIGHT, key : {}, state : {}, gain_light : {}, gain_dec : {}, gain_inc : {}".format(key, state, self.msg.buttons[5], self.msg.buttons[3], self.msg.buttons[4]))

    def pwm_set_neutral(self,pwm):
        """Set to zeros when joystisk come back to the center"""
        if pwm >= 1495 and pwm <= 1505: #to ensure that when stick are in the center neutral pwm i send
            pwm = self.pwm_neutral
        return int(pwm)

    def publish(self):
        try:
            events = get_gamepad() #from the input library
            for event in events:
                #print(event.ev_type, event.code, event.state)
                if event.code in self.input:
                    self.input[event.code](event.code,event.state) #launch the method that correspond to the input 
        except inputs.UnpluggedError:
            print("inouts.UnpluggedError : No gamepad found")
            sys.exit('1')
        except e as error:
            print(e)
            sys.exit('1')
        print("LIST_BUTTONS_CLICKED : {}".format(self.list_buttons_clicked))
        self.msg_header()
        self.pub.publish(self.msg)
 
if __name__ == "__main__":
    rospy.init_node('Gamepad', anonymous=True)

    gamepad = Gamepad()

    while not rospy.is_shutdown():
       gamepad.publish()

