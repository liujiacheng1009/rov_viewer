#!/usr/bin/env python3
from PyQt5 import QtGui, QtCore, QtWidgets, uic
# from PySide2.QtWidgets import QApplication, QMainWindow, QFileDialog, QListWidgetItem, QMessageBox,QTextEdit,QVBoxLayout
import sys,os,time
import rospy
from std_msgs.msg import Bool ,String ,UInt16
from sensor_msgs.msg import BatteryState,Image, Imu,Joy
from rov_viewer.msg import Attitude, Bar30, State,SetHeading,SetDepth,SetVelocity ## 自定义msg
import signal
from cv_bridge import CvBridge
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from pid import PID
from gui2_windows import Ui_MainWindow

PATH = "/home/bluerov/Downloads/test/catkin_ws/src/rov_viewer/src/gui/"

g = 9.81  # m.s^-2 gravitational acceleration 
p0 = 990*100 #Pa surface pressure NEED to be cheked
rho = 1000 # kg/m^3  water density

class Display(QtWidgets.QMainWindow):
    def __init__(self):
        super(Display, self).__init__()
        #uic.loadUi(PATH +"/gui2.ui", self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("ROV VIEWER")
        self.ROV_name = '/BlueRov2'
        #self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.init_param()
        self.set_ros_subs()
        self.set_ros_pubs()
        self.setWindowFlags(QtCore.Qt.WindowSystemMenuHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint)
        self.init_page()
        self.init_page2()
        self.init_page3()
        self.init_page4()
        self.init_imu_canvas()

        self.ui.btn_imu.clicked.connect(self.go_to_imu_viewer)
        self.ui.btn_cam.clicked.connect(self.go_to_cam_viewer)
        self.ui.btn_manual.clicked.connect(self.go_to_manual_ctr)
        self.ui.btn_depth.clicked.connect(self.go_to_depth_crl)
        self.ui.btn_velo.clicked.connect(self.go_to_velo_crl)
        self.ui.btn_atti.clicked.connect(self.go_to_atti_crl)
        self.ui.btn_arm.clicked.connect(self.send_arm)
        ## 定向、定速、定深控制按钮
        self.ui.pb_depth_stop.clicked.connect(self.exit_depth_ctr)
        self.ui.pb_depth_send.clicked.connect(self.set_depth_ctr)
        self.ui.pb_heading_stop.clicked.connect(self.exit_heading_ctr)
        self.ui.pb_heading_send.clicked.connect(self.set_heading_ctr)
        self.ui.pb_velocity_stop.clicked.connect(self.exit_velocity_ctr)
        self.ui.pb_velocity_send.clicked.connect(self.set_velocity_ctr)
        ## 定深页面
        self.ui.dsb_depth_target.valueChanged.connect(self.depthTargetValuechange)
        self.ui.sb_depth_max_pwm.valueChanged.connect(self.depthMaxPWMValuechange)
        self.ui.sb_depth_ki.valueChanged.connect(self.depthKIValuechange)
        self.ui.sb_depth_kp.valueChanged.connect(self.depthKPValuechange)
        self.ui.sb_depth_kd.valueChanged.connect(self.depthKDValuechange)
        ## 定向页面, I参数无效
        self.ui.dsb_heading_target.valueChanged.connect(self.headingTargetValuechange)
        self.ui.sb_heading_max_pwm.valueChanged.connect(self.headingMaxPWMValuechange)
        self.ui.sb_heading_kp.valueChanged.connect(self.headingKPValuechange)
        self.ui.sb_heading_kd.valueChanged.connect(self.headingKDValuechange)

        ## 定速页面, I参数无效
        self.ui.dsb_velocity_target.valueChanged.connect(self.velocityTargetValuechange)
        self.ui.sb_velocity_max_pwm.valueChanged.connect(self.velocityMaxPWMValuechange)
        self.ui.sb_velocity_kp.valueChanged.connect(self.velocityKPValuechange)
        self.ui.sb_velocity_kd.valueChanged.connect(self.velocityKDValuechange)

        self.timer = QtCore.QTimer() ## 定时刷新
        self.timer.timeout.connect(self.display)
        self.timer.start(250)

    def velocityTargetValuechange(self):
        self.set_velocity.velocity_desired = self.ui.dsb_velocity_target.value()

    def velocityMaxPWMValuechange(self):
        self.set_velocity.pwm_max = self.ui.sb_velocity_max_pwm.value()
    
    def velocityKPValuechange(self):
        self.set_velocity.KP = self.ui.sb_velocity_kp.value()

    def velocityKDValuechange(self):
        self.set_velocity.KD = self.ui.sb_velocity_kd.value()

    def headingTargetValuechange(self):
        self.set_heading.heading_desired = self.ui.dsb_heading_target.value()

    def headingMaxPWMValuechange(self):
        self.set_heading.pwm_max = self.ui.sb_heading_max_pwm.value()

    def headingKPValuechange(self):
        self.set_heading.KP = self.ui.sb_heading_kp.value()

    def headingKDValuechange(self):
        self.set_heading.KD = self.ui.sb_heading_kd.value()

    def exit_heading_ctr(self):
        self.set_heading.enable_heading_ctrl = False

    def set_heading_ctr(self):
        self.set_heading.enable_heading_ctrl = True

    def exit_velocity_ctr(self):
        self.set_velocity.enable_velocity_ctrl = False

    def set_velocity_ctr(self):
        self.set_velocity.enable_velocity_ctrl = True

    def depthTargetValuechange(self):
        self.set_depth.depth_desired = self.ui.dsb_depth_target.value()

    def depthMaxPWMValuechange(self):
        self.set_depth.pwm_max = self.ui.sb_depth_max_pwm.value()

    def depthKIValuechange(self):
        self.set_depth.KI = self.ui.sb_depth_ki.value()
    
    def depthKPValuechange(self):
        self.set_depth.KP = self.ui.sb_depth_kp.value()

    def depthKDValuechange(self):
        self.set_depth.KD = self.ui.sb_depth_kd.value()

    def exit_depth_ctr(self):
        self.set_depth.enable_depth_ctrl = False
    
    def set_depth_ctr(self):
        self.set_depth.enable_depth_ctrl = True

    def set_ros_pubs(self):
        self.pub_set_arm = rospy.Publisher(self.ROV_name+'/Setting/arm', Bool, queue_size=10)
        self.pub_set_depth = rospy.Publisher(self.ROV_name+'/Setting/set_depth', SetDepth, queue_size=10)## PID定深
        self.pub_set_heading = rospy.Publisher(self.ROV_name+'/Setting/set_heading', SetHeading, queue_size=10)## PID定向
        self.pub_set_velocity = rospy.Publisher(self.ROV_name+'/Setting/set_velocity', SetVelocity, queue_size=10)## PID定速

    def send_arm(self):
        if(self.arm != 0):
            self.pub_set_arm.publish(0)
        else:
            self.pub_set_arm.publish(1)


    def init_imu_canvas(self):
        self.acc_z = []
        self.acc_x = []
        self.acc_y = []
        self.angv_z = []
        self.angv_x = []
        self.angv_y = []
        self.t = range(0,200)
        self.imuPlotLayout = QtWidgets.QVBoxLayout(self.ui.listWidget_imu)
        self.figure = Figure()
        self.axes1 = self.figure.add_subplot(321)
        self.axes2 = self.figure.add_subplot(322)
        self.axes3 = self.figure.add_subplot(323)
        self.axes4 = self.figure.add_subplot(324)
        self.axes5 = self.figure.add_subplot(325)
        self.axes6 = self.figure.add_subplot(326)
        self.canvas = FigureCanvas(self.figure)
        self.imuPlotLayout.addWidget(self.canvas)
        #self.ui.listWidget_imu.setFocus()

    def go_to_imu_viewer(self):
        self.ui.tabWidget.setCurrentIndex(1)

    def go_to_cam_viewer(self):
        self.ui.tabWidget.setCurrentIndex(2)

    def go_to_manual_ctr(self):
        self.ui.tabWidget.setCurrentIndex(3)

    def go_to_depth_crl(self):
        self.ui.tabWidget.setCurrentIndex(4)

    def go_to_atti_crl(self):
        self.ui.tabWidget.setCurrentIndex(5)

    def go_to_velo_crl(self):
        self.ui.tabWidget.setCurrentIndex(6)
    
    def init_param(self):
        self.mode = "NOT CLEAR"
        self.battery = 0
        self.arm = -1
        self.imu = [0,0,0,0,0,0]
        self.bridge = CvBridge()
        self.image = None
        self.light = -1
        self.cam_angle = -1

        self.set_depth = SetDepth()
        self.set_depth.enable_depth_ctrl = False
        self.set_depth.pwm_max = 1700
        self.set_depth.KI = 100
        self.set_depth.KP = 600
        self.set_depth.KD = 50

        self.set_heading = SetHeading()
        self.set_heading.enable_heading_ctrl = False
        self.set_heading.pwm_max = 1700
        self.set_heading.KP = 35
        self.set_heading.KD = 25

        self.set_velocity = SetVelocity()
        self.set_velocity.enable_velocity_ctrl = False
        self.set_velocity.pwm_max = 1700
        self.set_velocity.KP = 100
        self.set_velocity.KD = 25


    def init_page(self):
        self.textEdit  = QtWidgets.QTextEdit()
        text = ''
        text += '图像大小： '+'未知'
        text += '\n'
        text += '帧率： ' +'未知'
        self.textEdit.setPlainText(text)
        layout=QtWidgets.QVBoxLayout(self.ui.page)
        layout.addWidget(self.textEdit)

    def init_page2(self):
        self.textEdit2  = QtWidgets.QTextEdit()
        text = ''
        text += 'Arm： '+'未知'
        text += '\n'
        text += '模式： ' +'未知'
        text += '\n'
        text += '电量： ' +'未知'
        self.textEdit2.setPlainText(text)
        layout=QtWidgets.QVBoxLayout(self.ui.page_2)
        layout.addWidget(self.textEdit2)

    def init_page3(self):
        self.textEdit3  = QtWidgets.QTextEdit()
        text = ''
        text += 'angular_velocity: \n'
        text += '未知\n'
        text += '未知\n'
        text += '未知\n'
        text += 'linear_acceleration\n'
        text += '未知\n'
        text += '未知\n'
        text += '未知\n'
        self.textEdit3.setPlainText(text)
        layout=QtWidgets.QVBoxLayout(self.ui.page_3)
        layout.addWidget(self.textEdit3)

    def init_page4(self):
        self.textEdit4  = QtWidgets.QTextEdit()
        text = ''
        text += '灯亮度: \n'
        text += '未知\n'
        text += '云台角度:\n'
        text += '未知\n'
        self.textEdit4.setPlainText(text)
        layout=QtWidgets.QVBoxLayout(self.ui.page_4)
        layout.addWidget(self.textEdit4)


    def set_ros_subs(self):
        rospy.Subscriber('/BlueRov2/state', State, self._state_callback) 
        rospy.Subscriber('/BlueRov2/battery', BatteryState, self._battery_callback)
        rospy.Subscriber('/BlueRov2/camera/image_raw',Image , self._image_callback)
        rospy.Subscriber('/BlueRov2/imu/data',Imu , self._imu_callback)

    def _state_callback(self,msg):
        self.mode = msg.mode 
        self.arm = 1 if(msg.arm) else 0
        self.light = msg.light
        self.cam_angle = msg.camera

    def _battery_callback(self, msg):
        self.battery = msg.voltage

    def _image_callback(self, msg):
        '''
        ros msg 转化为cv image
        '''
        self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

    def _imu_callback(self,msg):
        self.imu = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.angv_x.append(self.imu[0])
        self.angv_y.append(self.imu[1])
        self.angv_z.append(self.imu[2])
        self.acc_x.append(self.imu[3])
        self.acc_y.append(self.imu[4])
        self.acc_z.append(self.imu[5])
        if(len(self.acc_z)>200):
            del self.angv_x[0]
            del self.angv_y[0]
            del self.angv_z[0]
            del self.acc_x[0]
            del self.acc_y[0]
            del self.acc_z[0]

    def display(self):
        self.update_page()
        self.update_page2()
        self.update_page3()
        self.update_page4()
        self.update_imu_plot()
        self.update_image_show()
        self.update_depth_ctr()
        self.update_heading_ctr()
        self.update_velocity_ctr()

    def update_heading_ctr(self):
        print(self.set_heading)
        self.pub_set_heading.publish(self.set_heading)

    def update_velocity_ctr(self):
        self.pub_set_velocity.publish(self.set_velocity)

    def update_depth_ctr(self):
        self.pub_set_depth.publish(self.set_depth)

    def update_image_show(self):
        if(self.image is None):
            return
        image = QtGui.QImage(self.image.data, self.image.shape[1], self.image.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
        # #self.videoLabel.setPixmap(QtGui.QPixmap.fromImage(qt_image))
        # if(self.play_video):
        self.ui.label_img_show.setPixmap(QtGui.QPixmap.fromImage(image))


    def update_imu_plot(self):
        if(len(self.acc_z)==len(self.t)):
            self.axes1.cla()
            self.axes2.cla()
            self.axes3.cla()
            self.axes4.cla()
            self.axes5.cla()
            self.axes6.cla()
            self.axes1.plot(self.t, self.angv_x, 'r')
            self.axes2.plot(self.t, self.angv_y, 'g')
            self.axes3.plot(self.t, self.angv_z, 'y')
            self.axes4.plot(self.t, self.acc_x, 'r')
            self.axes5.plot(self.t, self.acc_y, 'g')
            self.axes6.plot(self.t, self.acc_z, 'y')
            self.canvas.draw()

    def update_page(self):
        text = ''
        if(self.image is None):
            text += '图像大小： '+'未知'
        else:
            text += '图像大小： '+ str(self.image.shape[0]) + 'x'+str(self.image.shape[1]) 
        text += '\n'
        text += '帧率： ' +'未知'
        self.textEdit.setPlainText(text)

    def update_page2(self):
        '''
        更新主页左侧的ROV状态，包括Arm,模式，电量
        '''
        text = ''
        if(self.arm==-1):
            text += 'Arm： '+'未知'
        elif(self.arm==0):
            text += 'Arm： '+'锁定'
        else:
            text += 'Arm： '+'解锁'
        text += '\n'
        if(self.mode=='MANUAL'):
            text += '模式： ' +'手动'
        else:
            text += '模式： ' + '未知'
        text += '\n'
        if(self.battery==0):
            text += '电量： ' +'未知'
        else:
            text += '电量： ' + str(round(self.battery,2)) ## 保留两位小数
        self.textEdit2.setPlainText(text)

    def update_page3(self):
        '''
        更新主页左侧的IMU信息，包括角加速度和线加速度
        '''
        text = ''
        text += 'angular_velocity: \n'
        text += str(round(self.imu[0],2)) +'\n'
        text += str(round(self.imu[1],2)) +'\n'
        text += str(round(self.imu[2],2)) +'\n'
        text += 'linear_acceleration\n'
        text += str(round(self.imu[3],2)) +'\n'
        text += str(round(self.imu[4],2)) +'\n'
        text += str(round(self.imu[5],2)) +'\n'
        self.textEdit3.setPlainText(text)

    def update_page4(self):
        '''
        更新主页左侧的其他信息，包括灯亮度和云台角度
        '''
        text = ''
        text += '灯亮度: \n'
        if(self.light==-1):
            text += '未知\n'
        else:
            text += str(round(self.light,2)) +'\n'
        text += '云台角度:\n'
        if(self.cam_angle==-1):
            text += '未知\n'
        else:
            text += str(round(self.cam_angle,2)) +'\n'
        self.textEdit4.setPlainText(text)

if __name__ == "__main__":
    rospy.init_node('gui2', anonymous=True)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = Display()
    window.show()
    app.exec_()