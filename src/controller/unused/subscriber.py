#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

def callback(data):
    print(data)

        
def listener():
    rate = rospy.Rate(10)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('TEST')
    rospy.Subscriber("/Command/joy", Joy, callback)
    listener()
