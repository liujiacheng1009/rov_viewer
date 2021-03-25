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
# from rov_viewer.msg import Attitude, Bar30, State,SetHeading,SetDepth,SetVelocity ## 自定义msg
import signal
from cv_bridge import CvBridge
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from multiprocessing import Process

from begin_page import BeginPage,ConnWidget,ROVInfo
from imu_page.imu_page import IMUPage
from image_page.image_page import ImagePage
from gamepad_page.gamepad_page import ManualCtrPage
from pid_ctr_page import PIDCtrPage


class GUI(QWidget):
    def __init__(self,parent=None):
        super(GUI,self).__init__(parent)
        self.setWindowTitle("ROV VIEWER")
        self.resize(960,640)
        self.rov_info = ROVInfo()
        self.begin_page = BeginPage(self)
        self.conn_widget = ConnWidget(self)
        self.imu_page = IMUPage()
        self.image_page = ImagePage()
        self.manual_ctr_page = ManualCtrPage()
        self.pid_ctr_page = PIDCtrPage()
        self.bluerov = None
        self.tabWidget = QTabWidget()
        widget1 = self.begin_page.getWidget()
        widget2 = self.imu_page.getWidget()
        widget3 = self.image_page.getWidget()
        widget4 = self.manual_ctr_page.getWidget()
        widget5 = self.pid_ctr_page.getWidget()
        widget6 = QWidget()
        self.tabWidget.addTab(widget1,QIcon(":/icon/icon/home.png"),"")
        self.tabWidget.addTab(widget2,"IMU")
        self.tabWidget.addTab(widget3,"相机")
        self.tabWidget.addTab(widget4,"手动控制")
        self.tabWidget.addTab(widget5,"PID控制")
        self.tabWidget.addTab(widget6,"自主导航")
        layout = QGridLayout()
        layout.addWidget(self.rov_info.getWidget(),0,0,8,1)
        layout.addWidget(self.conn_widget.creatConnGroup(),0,1,1,4)
        layout.addWidget(self.tabWidget,1,1,7,4)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 8)
        # layout.setRowStretch(0,1)
        # layout.setRowStretch(1,3)
        self.setLayout(layout)

        ## 从ros topic 获取数据
        #rospy.Subscriber('/BlueRov2/imu/data',Imu,self.imu_callback)
        rospy.Subscriber('/cam0/image_raw', Image, self.camera_callback)
        
        ## 获取的数据类型
        self.imu = None
        self.image = None
        self.bridge = CvBridge()
        # self.rov_proc = Process(target=self.bluerovProcess,args=())
        # self.rov_proc.start()

    def bluerovProcess(self):
         while not rospy.is_shutdown():
            if(self.bluerov is not None):
                try:
                    self.bluerov.publish()
                    print("hh")
                except:
                    pass
                


    def imu_callback(self,msg):
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyr = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        self.imu_page.dataplot.updateSensorData(acc,gyr,[1,1,1])
    
    def camera_callback(self,msg):
        id = self.image_page.tab_wd.currentIndex()
        self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        if(id==0):
            qimg = QImage(self.image.data, self.image.shape[1], self.image.shape[0], QImage.Format_RGB888)
            self.image_page.image_label.setPixmap(QPixmap.fromImage(qimg))
        elif(id==2):
            self.image_page.yolo_wd.callback(self.image)

    

if __name__ == "__main__":
    rospy.init_node("gui")
    glutInit(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    gui = GUI()
    gui.show()
    app.exec_()