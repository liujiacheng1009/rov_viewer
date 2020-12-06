#!/usr/bin/env python3

import rospy
from bluerov_ros_playground.msg import Set_depth
from bluerov_ros_playground.msg import Set_heading
from bluerov_ros_playground.msg import Set_velocity
from bluerov_ros_playground.msg import Set_target


def set_msg_depth(pwm_max, KI, KP, KD):
	msg_depth = Set_depth()
	msg_depth.pwm_max = pwm_max
	msg_depth.KI= KI
	msg_depth.KP= KP
	msg_depth.KD= KD
	return msg_depth

def set_msg_heading(pwm_max, KP, KD):
	msg_heading = Set_heading()
	msg_heading.pwm_max = pwm_max
	msg_heading.KP = KP
	msg_heading.KD = KD
	return msg_heading

def set_msg_velocity(pwm_max, KP, KD):
	msg_velocity = Set_velocity()
	msg_velocity.pwm_max = pwm_max
	msg_velocity.KP = KP
	msg_velocity.KD = KD
	return msg_velocity

def set_msg_attitude(depth_desired, heading_desired, velocity_desired):
        msg_attitude = Set_attitude()
        msg_attitude.depth_desired = depth_desired
        msg_attitude.heading_desired = heading_desired
        msg_attitude.velocity_desired = velocity_desired
        return msg_attitude

if __name__ == "__main__":
	rospy.init_node('Setter', anonymous=True)
	rate = rospy.Rate(4)
	pub_depth = rospy.Publisher('/Settings/set_depth', Set_depth, queue_size=10)
	pub_heading = rospy.Publisher('/Settings/set_heading', Set_heading, queue_size=10)
	pub_velocity = rospy.Publisher('/Settings/set_velocity', Set_velocity, queue_size=10)
    pub_target = rospy.Publisher('/Settings/set_target', Set_target, queue_size=10)

	msg_depth = set_msg_depth(1650,  100, 600, 50)
	msg_heading = set_msg_heading(1550,  35, 25)
	msg_velocity = set_msg_velocity(1550,  100, 25)
    msg_target = set_msg_target(-0.2,  2.96,  1.)
	while not rospy.is_shutdown():
		pub_depth.publish(msg_depth)
		pub_heading.publish(msg_heading)
		pub_velocity.publish(msg_velocity)
		pub_target.publish(msg_target)
		rate.sleep()
