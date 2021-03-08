#!/usr/bin/env python3
from PyQt5 import QtGui, QtCore, QtWidgets, uic
import sys,os
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from rov_viewer.msg import Attitude, Bar30, State ## 自定义msg
import signal

PATH = "/home/bluerov/Downloads/catkin_ws/src/rov_viewer/src/gui/"

class Display(QtWidgets.QMainWindow):
    def __init__(self):
        super(Display, self).__init__()
        uic.loadUi(PATH +"/mainwindow.ui", self)
        self.init_list_widget()
        self.init_param()
        self.set_ros_pub()
        self.set_ros_sub()
        self.timer = QtCore.QTimer() ## 定时刷新
        self.timer.timeout.connect(self.display)
        self.timer.start(250)

    def init_param(self):
        self.battery = 0
        self.state = State()
        self.imu = None
        self.image = None


    
    def init_list_widget(self):
        self.listWidget.insertItem(0,"参数设置") 
        self.listWidget.insertItem(1,"运动控制")
        self.listWidget.insertItem(2,"显示图像")
        self.listWidget.insertItem(3,"IMU数据")
        self.listWidget.currentRowChanged.connect(self.stackedWidget.setCurrentIndex)

    def set_ros_pub(self):
        pass

    def set_ros_sub(self):
        rospy.Subscriber('/BlueRov2/state', State, self._state_callback) 
        rospy.Subscriber('/BlueRov2/battery', BatteryState, self._battery_callback)
        rospy.Subscriber('/BlueRov2/bar30', Bar30, self._bar30_callback)

    def display(self):
        print(self.battery)
        if(self.battery):
            self.lcdNumber_vol.display(round(self.battery, 2))
        
        

    def _state_callback(self,msg):
        self.state = msg

    def _battery_callback(self, msg):
        print(msg.voltage)
        self.battery = msg.voltage

    def _bar30_callback(self,msg):
        self.bar30_pressure_measured = msg.press_abs


if __name__ == "__main__":
    rospy.init_node('gui', anonymous=True)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = Display()
    window.show()
    app.exec_()