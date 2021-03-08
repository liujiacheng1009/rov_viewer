#!/usr/bin/env python3
## 实现了将bluerov的IMU、相机、状态信息转化为ros msg
## BlueRov类继承自Bridge
from __future__ import division

import json
import math
import re
import rospy
import sys
import time

from mavlink import Bridge
from gstreamer import Video

# convert opencv image to ros image msg
from cv_bridge import CvBridge

# msgs type
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image, Imu
from std_msgs.msg import Bool, String ,UInt16
from rov_viewer.msg import Bar30, Attitude, State ## 自定义数据类型


class BlueRov(Bridge):
    def __init__(self, device='udp:192.168.2.1:14550', baudrate=115200):
        """ BlueRov ROS Bridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate)
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'

        self.video = Video()
        self.video_bridge = CvBridge()

        self.pub_topics = {
            '/battery':
            [
                self._create_battery_msg,
                BatteryState,
                1
            ],
            '/camera/image_raw':
            [
                self._create_camera_msg,
                Image,
                1
            ],
            '/state':
            [
                self._create_ROV_state,
                State,
                1
            ],
            '/imu/data':
            [
                self._create_imu_msg,
                Imu,
                1
            ],
            '/bar30':
            [
                self._create_bar30_msg,
                Bar30,
                1
            ]
        }

        self.mavlink_msg_available = {}
        for topic, pubs in self.pub_topics.items():
            self.mavlink_msg_available[topic] = 0
            pubs.append(rospy.Publisher(self.ROV_name+topic, pubs[1], queue_size=pubs[2]))

    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    def _create_bar30_msg(self,topic):
        if 'SCALED_PRESSURE2' not in self.get_data():
            raise Exception('no SCALE_PRESSURE2 data')
        else :
            pass
        bar30_data = self.get_data()['SCALED_PRESSURE2']
        msg = Bar30()
        self._create_header(msg)
        msg.time_boot_ms = bar30_data['time_boot_ms']
        msg.press_abs    = bar30_data['press_abs']
        msg.press_diff   = bar30_data['press_diff']
        msg.temperature  = bar30_data['temperature']

        self.pub_topics[topic][3].publish(msg)

    def _create_imu_msg(self,topic):
        """ Create imu message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: move all msgs creating to msg
        msg = Imu()

        self._create_header(msg)

        #http://mavlink.org/messages/common#SCALED_IMU
        imu_data = None
        for i in ['', '2', '3']:
            try:
                imu_data = self.get_data()['SCALED_IMU{}'.format(i)]
                break
            except Exception as e:
                pass

        if imu_data is None:
            raise Exception('no SCALED_IMUX data')

        acc_data = [imu_data['{}acc'.format(i)]  for i in ['x', 'y', 'z']]
        gyr_data = [imu_data['{}gyro'.format(i)] for i in ['x', 'y', 'z']]
        mag_data = [imu_data['{}mag'.format(i)]  for i in ['x', 'y', 'z']]

        #http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
        msg.linear_acceleration.x = acc_data[0]/100
        msg.linear_acceleration.y = acc_data[1]/100
        msg.linear_acceleration.z = acc_data[2]/100
        msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg.angular_velocity.x = gyr_data[0]/1000
        msg.angular_velocity.y = gyr_data[1]/1000
        msg.angular_velocity.z = gyr_data[2]/1000
        msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.orientation.w = cy * cr * cp + sy * sr * sp
        msg.orientation.x = cy * sr * cp - sy * cr * sp
        msg.orientation.y = cy * cr * sp + sy * sr * cp
        msg.orientation.z = sy * cr * cp - cy * sr * sp

        msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.pub_topics[topic][3].publish(msg)

    def _create_battery_msg(self,topic):
        """ Create battery message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SYS_STATUS' not in self.get_data():
            raise Exception('no SYS_STATUS data')

        if 'BATTERY_STATUS' not in self.get_data():
            raise Exception('no BATTERY_STATUS data')

        bat = BatteryState()
        self._create_header(bat)

        #http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
        bat.voltage = self.get_data()['SYS_STATUS']['voltage_battery']/1000
        bat.current = self.get_data()['SYS_STATUS']['current_battery']/100
        bat.percentage = self.get_data()['BATTERY_STATUS']['battery_remaining']/100
        self.pub_topics[topic][3].publish(bat)

    def _create_camera_msg(self,topic):
        if not self.video.frame_available():
            return
        frame = self.video.frame()
        image_msg = Image()
        self._create_header(image_msg)
        height, width, channels = frame.shape
        image_msg.width = width
        image_msg.height = height
        image_msg.encoding = 'bgr8'
        image_msg.data = frame
        msg = self.video_bridge.cv2_to_imgmsg(frame, "bgr8")
        self._create_header(msg)
        msg.step = int(msg.step)
        self.pub_topics[topic][3].publish(msg)
    

    def _create_ROV_state(self,topic):
        """ Create ROV state message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SERVO_OUTPUT_RAW' not in self.get_data():
            raise Exception('no SERVO_OUTPUT_RAW data')

        if 'HEARTBEAT' not in self.get_data():
            raise Exception('no HEARTBEAT data')

        servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
        servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i+1)] for i in range(8)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(6)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle/400
            else:
                throttle = throttle/500

        light_on = (servo_output_raw[6] - 1100) / 8
        #need to check
        camera_angle = servo_output_raw[7] - 1500

        # Create angle from pwm
        camera_angle = -45*camera_angle/400

        base_mode = self.get_data()['HEARTBEAT']['base_mode']
        custom_mode = self.get_data()['HEARTBEAT']['custom_mode']

        mode, arm = self.decode_mode(base_mode, custom_mode)

        data = State()
        data.arm = arm
        data.rc1 = motor_throttle[0]
        data.rc2 = motor_throttle[1]
        data.rc3 = motor_throttle[2]
        data.rc4 = motor_throttle[3]
        data.rc5 = motor_throttle[4]
        data.rc6 = motor_throttle[5]
        data.light = light_on
        data.camera = camera_angle
        data.mode = mode
        
        self.pub_topics[topic][3].publish(data)

    def publish(self):
        """ Publish the data in ROS topics
        """
        self.update()
        for topic, pubs in self.pub_topics.items():
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    pubs[0](topic)
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    bluerov = BlueRov(device='udp:192.168.2.1:14560')

    while not rospy.is_shutdown():
        bluerov.publish()
