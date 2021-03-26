import os 
import cv2 
import cv_bridge
import rospy 
from sensor_msgs.msg import Image

path = "iamges/"
class Simulate:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.pub = rospy.Publisher('/camera/image', Image, queue_size=10)
        self.image_path = os.listdir(path)

    def start(self):
        while not rospy.is_shutdown():
            for p in self.image_path:
                image = cv2.imread(path+p)
                self.pub.publish(self.bridge.cv2_to_imgmsg(image,'bgr8'))
                self.loop_rate.sleep()



if __name__ == "__main__":
    rospy.init_node("simulate")
    s = Simulate()
    s.start()