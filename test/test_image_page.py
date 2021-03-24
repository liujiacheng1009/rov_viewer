#!/usr/bin/env python3
import sys 
import os 
curr_dir = os.getcwd()
sys.path.append(curr_dir +"/src/gui/image")

from yolo_wd import YOLOWd


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.init_node("yolo")
    app = QApplication(sys.argv)
    w = YOLOWd()
    w.show()
    rospy.Subscriber('/cam0/image_raw', Image, w.callback)
    app.exec_()
