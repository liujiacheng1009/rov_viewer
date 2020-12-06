#!/usr/bin/env python3

import bluerov_node as BR
import rospy

try:
    rospy.init_node('user_node', log_level=rospy.DEBUG)
except rospy.ROSInterruptException as error:
    print('pubs error with ROS: ', error)
    exit(1)

device = rospy.get_param("/node/device", 'udp:localhost:14550')
print('Running with device', device)
bluerov = BR.BlueRov(device=device)

while not rospy.is_shutdown():
    bluerov.publish()