#!/usr/bin/env python3
from PyQt5 import QtGui, QtCore, QtWidgets, uic

import sys,os,time
import rospy
from std_msgs.msg import Bool ,String ,UInt16
from sensor_msgs.msg import BatteryState,Image, Imu,Joy
from rov_viewer.msg import Attitude, Bar30, State,Set_target,Set_heading,Set_depth,Set_velocity ## 自定义msg
import signal
from cv_bridge import CvBridge
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from pid import PID

PATH = "/home/bluerov/Downloads/catkin_ws/src/rov_viewer/src/gui/"

g = 9.81  # m.s^-2 gravitational acceleration 
p0 = 990*100 #Pa surface pressure NEED to be cheked
rho = 1000 # kg/m^3  water density

class Display(QtWidgets.QMainWindow):
    def __init__(self):
        super(Display, self).__init__()
        uic.loadUi(PATH +"/mainwindow.ui", self)
        self.setWindowTitle("ROV VIEWER")
        self.ROV_name = '/BlueRov2'
        ##self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setWindowFlags(QtCore.Qt.WindowSystemMenuHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint)
        self.init_list_widget()
        self.init_imu_canvas()
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
        self.bar30_data = []
        self.imu = None
        self.image = None
        self.bridge = CvBridge()
        self.play_video = False
       # self.pid = PID()
        self.target_ctrl_msgToSend = Set_target()
        self.heading_ctrl_msgToSend = Set_heading() 
        self.depth_ctrl_msgToSend = Set_depth()
        self.velocity_ctrl_msgToSend = Set_velocity()
        self._enable_publish_target = False


    def init_imu_canvas(self):
        self.plot_imu = False
        self.acc_z = []
        self.acc_x = []
        self.acc_y = []
        self.t = range(0,200)
        self.imuPlotLayout = QtWidgets.QVBoxLayout(self.imuWidget)
        self.figure = Figure()
        self.axes1 = self.figure.add_subplot(311)
        self.axes2 = self.figure.add_subplot(312)
        self.axes3 = self.figure.add_subplot(313)
        self.canvas = FigureCanvas(self.figure)
        self.imuPlotLayout.addWidget(self.canvas)
        self.imuWidget.setFocus()
        
    
    def add_callbacks(self):
        self.videoPushButton.clicked.connect(self.onVideoPushButtonClicked)
        self.videoExitPushButton.clicked.connect(self.onVideoExitPushButtonClicked)
        self.imuPushButton.clicked.connect(self.onImuPushButtonClicked)
        self.imuExitPushButton.clicked.connect(self.onImuExitPushButtonClicked)
        self.comboBox_lock.currentIndexChanged.connect(self.lockSelectionchange)
        self.comboBox_mode.currentIndexChanged.connect(self.modeSelectionchange)
        #self.depth_ctr_pb.clicked.connect(self.onDepthCtrPBClicked)
        self.pushButton_send_parameters_target.clicked.connect(self._target_param_clicked)
        self.pushButton_send_parameters_heading.clicked.connect(self._heading_param_clicked)
        self.pushButton_send_parameters_velocity.clicked.connect(self._velocity_param_clicked)
        self.pushButton_send_parameters_depth.clicked.connect(self._depth_param_clicked)

    def _depth_param_clicked(self):
        """Raised when button SEND is clicked from depth PID parameters
        Define depth PID coefficients for message Set_depth
        """
        self.depth_ctrl_msgToSend.KI = self.spinBox_KI_depth.value()
        self.depth_ctrl_msgToSend.KP = self.spinBox_KP_depth.value()
        self.depth_ctrl_msgToSend.KD = self.spinBox_KD_depth.value()

    def _heading_param_clicked(self):
        """Raised when button SEND is clicked from heading PID parameters
        Define heading PID coefficients for message Set_heading
        """
        #self.heading_ctrl_msgToSend.KI = self.spinBox_KI_heading.value()
        self.heading_ctrl_msgToSend.KP = self.spinBox_KP_heading.value()
        self.heading_ctrl_msgToSend.KD = self.spinBox_KD_heading.value()

    def _velocity_param_clicked(self):
        """Raised when button SEND is clicked from velocity PID parameters
        Define heading PID coefficients for message Set_velocity
        """
        #self.velocity_ctrl_msgToSend.KI = self.spinBox_KI_velocity.value()
        self.velocity_ctrl_msgToSend.KP = self.spinBox_KP_velocity.value()
        self.velocity_ctrl_msgToSend.KD = self.spinBox_KD_velocity.value()

    def _pwm_max_clicked(self):
        """Raised when button SEND is clicked from PWM MAX
        Define the max pwm for all controller for their saturation method
        """
        self.depth_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value() 
        self.heading_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value()
        self.velocity_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value()

    def _target_param_clicked(self):
        self.target_ctrl_msgToSend.depth_desired = - self.doubleSpinBox_depth.value()
        self.target_ctrl_msgToSend.heading_desired = self.doubleSpinBox_heading.value()
        self.target_ctrl_msgToSend.velocity_desired = self.doubleSpinBox_velocity.value()

    # def onDepthCtrPBClicked(self):
    #     self.pid.kp = self.spinBox_depth_p.value()
    #     self.pid.ki = self.spinBox_depth_i.value()
    #     self.pid.kd = self.spinBox_depth_d.value()
    #     if(not self.bar30_data):
    #         return
    #     curr_depth=  -(self.bar30_data[1]*100-p0)/(rho*g)
    #     if not self.depth :
    #         self.depth = curr_depth
    #         return
    #     if not self.time :
    #         self.time = self.bar30_data[0]
    #         return
    #     delta_value = curr_value - self.depth
    #     self.depth = curr_depth 
    #     delta_t = self.bar30_data[0] - self.time 
    #     self.time = self.bar30_data[0]
    #     target_depth = self.spinBox_target_depth.value()
    #     u = self.pid.control_pid(curr_depth,target_depth,delta_t)
    #     mavutil.mavfile.set_servo(self.conn, 9, u)


    def lockSelectionchange(self,idx):
        if(idx==0): ##锁定
            self.pub_set_arm.publish(0)
        elif (idx==1): ##解锁
            self.pub_set_arm.publish(1)
    def modeSelectionchange(self,idx):
        if(idx==0):
            self.pub_set_mode.publish("manual")
        elif (idx==1):
            self.pub_set_mode.publish("auto")

        

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
        self.pub_set_arm = rospy.Publisher(self.ROV_name+'/Setting/arm', Bool, queue_size=10)
        self.pub_set_mode = rospy.Publisher(self.ROV_name+'/Setting/mode/set', String, queue_size=10)
        self.pub_set_manual_control = rospy.Publisher(self.ROV_name+'/Setting/manual_control', Joy, queue_size=10)
        self.pub_set_heading = rospy.Publisher(self.ROV_name+'/Settings/set_heading', Set_heading, queue_size=10)
        self.pub_set_depth = rospy.Publisher(self.ROV_name+'/Settings/set_depth', Set_depth, queue_size=10)
        self.pub_set_velocity = rospy.Publisher(self.ROV_name+'/Settings/set_velocity', Set_velocity, queue_size=10)
        self.pub_set_target = rospy.Publisher(self.ROV_name+'/Settings/set_target', Set_target, queue_size=10)


    def set_ros_sub(self):
        rospy.Subscriber('/BlueRov2/state', State, self._state_callback) 
        rospy.Subscriber('/BlueRov2/battery', BatteryState, self._battery_callback)
        rospy.Subscriber('/BlueRov2/bar30', Bar30, self._bar30_callback)
        rospy.Subscriber('/BlueRov2/camera/image_raw',Image , self._image_callback)
        rospy.Subscriber('/BlueRov2/imu/data',Imu , self._imu_callback)
        


    def display(self):
        if(self.battery):
            self.lcdNumber_vol.display(round(self.battery, 2))

        self.pub_set_velocity.publish(self.velocity_ctrl_msgToSend)
        self.pub_set_heading.publish(self.heading_ctrl_msgToSend)
        self.pub_set_depth.publish(self.depth_ctrl_msgToSend)
        if self._enable_publish_target:
            self.pub_set_target.publish(self.target_ctrl_msgToSend)


    def _imu_callback(self,msg):
        self.acc_z.append(int(msg.linear_acceleration.z*100))
        self.acc_x.append(int(msg.linear_acceleration.x*100))
        self.acc_y.append(int(msg.linear_acceleration.y*100))

        if(len(self.acc_z)>200):
            del self.acc_z[0]
            del self.acc_y[0]
            del self.acc_x[0]
        else:
            self.plot_imu = False
        if(self.plot_imu):
            self.axes1.cla()
            self.axes2.cla()
            self.axes3.cla()
            self.axes1.plot(self.t, self.acc_z, 'r')
            self.axes2.plot(self.t, self.acc_y, 'g')
            self.axes3.plot(self.t, self.acc_x, 'y')
            self.canvas.draw()
            time.sleep(0.05)

        
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
        self.bar30_data = [ msg.time_boot_ms,msg.press_abs,
                            msg.press_diff,msg.temperature ]


if __name__ == "__main__":
    rospy.init_node('gui', anonymous=True)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = Display()
    window.show()
    app.exec_()