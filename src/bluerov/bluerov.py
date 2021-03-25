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

from mav_bridge import MAVBridge
from gst_reader import GSTReader

from cv_bridge import CvBridge

# msgs type
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image, Imu,Joy
from std_msgs.msg import Bool, String ,UInt16
from rov_viewer.msg import Bar30, Attitude, State ## 自定义数据类型


class BlueRov(MAVBridge):
    def __init__(self, device='udp:192.168.2.1:14560', baudrate=115200):
        """ 继承自MAVBridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate)
        self.ROV_name = '/BlueRov2'
        self.model_base_link = '/base_link'

        self.video = GSTReader()
        self.video_bridge = CvBridge()
        ##这里可以获得的信号有电池电量、
        ## 图像、IMU、自定义ROV state、深度计、自定义attitude
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
            ],
            '/imu/attitude':
            [
                self._create_imu_euler_msg,
                Attitude,
                1
            ]
        }
        ## 手动控制，将gamepad的控制信号传输给ROV,这里其实只能控制灯光
        ## arm 用于解锁
        ## rc_channel可以用于模拟控制(轴向运动，而非控制电机)，不建议使用
        self.sub_topics= {
            '/Setting/manual_control':
            [
                self._manual_control_callback,
                Joy,
                1
            ],
            '/Setting/arm':
            [
                self._arm_callback,
                Bool,
                1
            ],
            '/rc_channel{}/set_pwm':
            [
                self._set_rc_channel_callback,
                UInt16,
                1,
                [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
            ]
        }
      
        self.mavlink_msg_available = {}
        for topic, pubs in self.pub_topics.items():
            self.mavlink_msg_available[topic] = 0 ## 记录故障时间戳
            pubs.append(rospy.Publisher(self.ROV_name+topic, pubs[1], queue_size=pubs[2]))
        for topic, subs in self.sub_topics.items():
            if(len(subs)==3):
                callback, msg_type, queue_size = subs
                rospy.Subscriber(self.ROV_name+topic, msg_type, callback, queue_size=queue_size)
            else:
                callback, msg_type, queue_size,arg = subs
                for name in arg:
                    topicx = topic.format(name)
                    rospy.Subscriber(self.ROV_name+topicx, msg_type, callback, callback_args=topicx, queue_size=queue_size)


    def _setpoint_velocity_cmd_vel_callback(self, msg, _):
        """ Set angular and linear velocity from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        params = [
            None,
            None,
            None,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            None,
            None,
            None,
            None,
            None,
            ]
        self.set_position_target_local_ned(params)

        #http://mavlink.org/messages/common#SET_ATTITUDE_TARGET
        params = [
            None,
            None,
            None,
            None,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
            None,
            ]
        self.set_attitude_target(params)
        
    def _set_servo_callback(self, msg, topic):
        """ Set servo from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            None: Description
        """
        paths = topic.split('/')
        servo_id = None
        for path in paths:
            if 'servo' in path:
                servo_id = int(re.search('[0-9]', path).group(0)) + 1
                # Found valid id !
                break
        else:
            # No valid id
            return
        print(servo_id)
        self.set_servo_pwm(servo_id, msg.data)

    def _set_rc_channel_callback(self, msg, topic):
        """ Set RC channel from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            TYPE: Description
        """
        paths = topic.split('/')
        channel_id = None
        for path in paths:
            if 'rc_channel' in path:
                channel_id = int(re.search('[0-9]', path).group(0))  - 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_rc_channel_pwm(channel_id, msg.data)

    def _set_mode_callback(self, msg):
        """ Set ROV mode from topic

        Args:
            msg (TYPE): Topic message
            _ (TYPE): Description
        """
        self.set_mode(msg.data)

    def _arm_callback(self, msg):
        """ Set arm state from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        self.arm_throttle(msg.data)

    def _manual_control_callback(self, msg):
        
        """ Set manual control message from topic
        
        Args:
            msg (TYPE): ROS message
            _ (TYPE): description
        """
        self.set_manual_control([0,0,0,0], msg.buttons)

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

    def _create_odometry_msg(self,topic):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """

        # Check if data is available
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = Odometry()

        self._create_header(msg)

        #http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.pose.position.x = xyz_data[0]
        msg.pose.pose.position.y = xyz_data[1]
        msg.pose.pose.position.z = xyz_data[2]
        msg.twist.twist.linear.x = vxyz_data[0]/100
        msg.twist.twist.linear.y = vxyz_data[1]/100
        msg.twist.twist.linear.z = vxyz_data[2]/100

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = orientation_speed[1]
        msg.twist.twist.angular.z = orientation_speed[2]

        self.pub_topics[topic][3].publish(msg)

    def _create_imu_euler_msg(self,topic):
        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')
        else :
            pass
        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]
        
        msg = Attitude()
        self._create_header(msg)
        msg.time_boot_ms = attitude_data['time_boot_ms']
        msg.roll = orientation[0]
        msg.pitch = orientation[1]
        msg.yaw = orientation[2]
        msg.rollspeed = orientation_speed[0]
        msg.pitchspeed = orientation_speed[1]
        msg.yawspeed = orientation_speed[2]

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
                ## 这里是串行处理，图像转换的时间较长，可能出现花屏
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    pubs[0](topic)
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)



if __name__ == '__main__':
    try:
        rospy.init_node('bluerov_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)


    bluerov = BlueRov(device='udp:192.168.2.1:14560')

    while not rospy.is_shutdown():
        bluerov.publish()
