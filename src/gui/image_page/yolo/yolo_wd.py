import rospy 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from cv_bridge import CvBridge
import sys,signal
from sensor_msgs.msg import Image
import cv2
import numpy as np 
from image_page.yolo.yolo import YOLO

class YOLOWd(QWidget):
    def __init__(self,parent=None):
        super(YOLOWd, self).__init__(parent)
        self.bridge = CvBridge()
        self.image = None
        self.yolo = YOLO()
        self.createWidget()
        self.resize(600,400)

    def createWidget(self):
        layout = QGridLayout()
        self.label = QLabel()
        layout.addWidget(self.label)
        self.setLayout(layout)

    def callback(self,msg):
        self.image = msg
        self.yolo.detect(self.image)
        image = QImage(self.image.data, self.image.shape[1], self.image.shape[0], QImage.Format_RGB888)
        self.label.setPixmap(QPixmap.fromImage(image))

 

# if __name__ == "__main__":
#     signal.signal(signal.SIGINT, signal.SIG_DFL)
#     rospy.init_node("yolo4_node")
#     app = QApplication(sys.argv)
#     w = YOLODetection()
#     w.show()
#     rospy.Subscriber('/cam0/image_raw', Image, w.callback)
#     app.exec_()
