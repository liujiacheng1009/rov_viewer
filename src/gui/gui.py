#!/usr/bin/env python3
'''
主界面
'''

from PyQt5 import QtGui, QtCore, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import sys,os,time
import rospy
from std_msgs.msg import Bool ,String ,UInt16
from sensor_msgs.msg import BatteryState,Image, Imu,Joy
from rov_viewer.msg import Attitude, Bar30, State,SetHeading,SetDepth,SetVelocity ## 自定义msg
import signal
from cv_bridge import CvBridge
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *


from rov_info import ROVInfo
from begin_page import BeginPage
from imu_page.main import IMUPage
from image_page import ImagePage
from manual_control_page import ManualCtrPage
from pid_ctr_page import PIDCtrPage
from conn import ConnWidget

class GUI(QWidget):
    def __init__(self,parent=None):
        super(GUI,self).__init__(parent)
        self.setWindowTitle("ROV VIEWER")
        self.resize(960,640)
        self.rov_info = ROVInfo()
        self.begin_page = BeginPage()
        self.conn_widget = ConnWidget()
        self.imu_page = IMUPage()
        self.image_page = ImagePage()
        self.manual_ctr_page = ManualCtrPage()
        self.pid_ctr_page = PIDCtrPage()
        tabWidget = QTabWidget()
        widget1 = self.begin_page.getWidget()
        widget2 = self.imu_page.getWidget()
        widget3 = self.image_page.getWidget()
        widget4 = self.manual_ctr_page.getWidget()
        widget5 = self.pid_ctr_page.getWidget()
        widget6 = QWidget()

        tabWidget.addTab(widget1,QIcon(":/icon/icon/home.png"),"")
        tabWidget.addTab(widget2,"IMU")
        tabWidget.addTab(widget3,"相机")
        tabWidget.addTab(widget4,"手动控制")
        tabWidget.addTab(widget5,"PID控制")
        tabWidget.addTab(widget6,"自主导航")
        layout = QGridLayout()
        layout.addWidget(self.rov_info.getWidget(),0,0,8,1)
        layout.addWidget(self.conn_widget.creatConnGroup(),0,1,1,4)
        layout.addWidget(tabWidget,1,1,7,4)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 8)
        # layout.setRowStretch(0,1)
        # layout.setRowStretch(1,3)
        self.setLayout(layout)


if __name__ == "__main__":
    glutInit(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    gui = GUI()
    gui.show()
    app.exec_()