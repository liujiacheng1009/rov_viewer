#!/usr/bin/env python3
from PyQt5 import QtGui, QtCore, QtWidgets, uic

import sys,os,time
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState,Image, Imu
from rov_viewer.msg import Attitude, Bar30, State ## 自定义msg
import signal
from cv_bridge import CvBridge
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

PATH = "/home/bluerov/Downloads/catkin_ws/src/rov_viewer/src/gui/"
i = 0 ## imu记时

class Display(QtWidgets.QMainWindow):
    def __init__(self):
        super(Display, self).__init__()
        uic.loadUi(PATH +"/mainwindow.ui", self)
        self.setWindowTitle("ROV VIEWER")
        ##self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setWindowFlags(QtCore.Qt.WindowSystemMenuHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint)
        self.init_list_widget()
        self.init_menubar()
        self.init_param()
        self.add_callbacks()
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
        self.bridge = CvBridge()
        self.play_video = False
        self.plot_imu = False
        self.acc_z = []
        self.t = []
        self.imuPlotLayout  = QtWidgets.QGridLayout(self.imuWidget)
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.imuPlotLayout.addWidget(self.canvas)

    
    def add_callbacks(self):
        self.videoPushButton.clicked.connect(self.onVideoPushButtonClicked)
        self.videoExitPushButton.clicked.connect(self.onVideoExitPushButtonClicked)
        self.imuPushButton.clicked.connect(self.onImuPushButtonClicked)
        self.imuExitPushButton.clicked.connect(self.onImuExitPushButtonClicked)

    def onVideoPushButtonClicked(self):
        self.play_video = True

    def onVideoExitPushButtonClicked(self):
        self.play_video = False
    
    def onImuPushButtonClicked(self):
        self.plot_imu = True

    def onImuExitPushButtonClicked(self):
        self.plot_imu = False

    def init_menubar(self):
        self.menuBar.addMenu('BlueROV')
        self.menuBar.addMenu('图像处理')
        self.menuBar.addMenu('点云处理')
        self.menuBar.addMenu('SLAM')
    
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
        rospy.Subscriber('/BlueRov2/camera/image_raw',Image , self._image_callback)
        rospy.Subscriber('/BlueRov2/imu/data',Imu , self._imu_callback)

    def display(self):
        if(self.battery):
            self.lcdNumber_vol.display(round(self.battery, 2))

    def _imu_callback(self,msg):
        global i
        self.acc_z.append(int(msg.linear_acceleration.z*100))
        self.t.append(i)
        print(int(msg.linear_acceleration.z*100))
        print(len(self.t),len(self.acc_z))
        
        if(len(self.acc_z)>=200):
            del self.acc_z[0]
            del self.t[0]
        if(self.plot_imu):
            self.figure.add_subplot(111).plot(self.t,self.acc_z)
            self.canvas.show()
            time.sleep(0.05)
        i+= 1
        
    def _image_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #print(cv_image.shape[1], cv_image.shape[0])
        self.image = QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
        #self.videoLabel.setPixmap(QtGui.QPixmap.fromImage(qt_image))
        if(self.play_video):
            self.videoLabel.setPixmap(QtGui.QPixmap.fromImage(self.image))

    def _state_callback(self,msg):
        self.state = msg

    def _battery_callback(self, msg):
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