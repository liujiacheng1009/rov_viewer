#from PySide2 import QtCore, QtGui,QtWidgets
from PyQt5 import QtCore 
from PyQt5 import QtWidgets 
from PyQt5 import QtGui 
#from conn import ConnWidget
import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from imu_page.visualization import Visualization
from imu_page.compass import Compass
from imu_page.dataplot import DataPlot
import rospy
from sensor_msgs.msg import Imu
import random

class IMUPage:
    def __init__(self):
        
        self.widget = QtWidgets.QWidget()
        self.visualization = Visualization()
        self.compass = Compass()
        self.dataplot = DataPlot()
        grid = QtWidgets.QGridLayout()
        self.label_acc_x = QLabel()
        self.label_acc_y = QLabel()
        self.label_acc_z = QLabel()
        self.label_gyr_x = QLabel()
        self.label_gyr_y = QLabel()
        self.label_gyr_z = QLabel()
        self.label_mag_x = QLabel()
        self.label_mag_y = QLabel()
        self.label_mag_z = QLabel()
        self.label_ang_x = QLabel()
        self.label_ang_y = QLabel()
        self.label_ang_z = QLabel()
        grid.addWidget(self.createVisualGroups(), 0,0,4,1)
        grid.addWidget(self.createAccDataGroupbox(),0,1)
        grid.addWidget(self.createGyrDataGroupbox(),1,1)
        grid.addWidget(self.createMagDataGroupbox(),2,1)
        grid.addWidget(self.createAngleGroupbox(),3,1)
        self.widget.setLayout(grid)
        rospy.Subscriber('/imu0',Imu,self.imu_callback)

    def imu_callback(self,msg):
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyr = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        self.label_acc_x.setText(str(acc[0]))
        self.label_acc_y.setText(str(acc[1]))
        self.label_acc_z.setText(str(acc[2]))
        self.label_gyr_x.setText(str(gyr[0]))
        self.label_gyr_y.setText(str(gyr[1]))
        self.label_gyr_z.setText(str(gyr[2]))
        self.visualization.updateAngles(random.randint(0,99),random.randint(0,120),random.randint(0,180))
        self.compass.updateAngle(random.randint(0,180))
        self.dataplot.updateSensorData(acc,gyr,[0,0,0])

    def createVisualGroups(self):
        groupBox = QtWidgets.QGroupBox()
        tabWidget = QtWidgets.QTabWidget()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)
        layout.addWidget(tabWidget)

        widget1 = self.visualization
        widget2 = self.compass
        widget3 = self.dataplot.createAnglePlot()
        widget4 = self.dataplot.createAccPlot()
        widget5 = self.dataplot.createGyrPlot()
        widget6 = self.dataplot.createMagPlot()
        tabWidget.addTab(widget1,"3D")
        tabWidget.addTab(widget2,"Compass")
        tabWidget.addTab(widget3,"Angles")
        tabWidget.addTab(widget4,"Acc")
        tabWidget.addTab(widget5,"Gyr")
        tabWidget.addTab(widget6,"Mag")

        return groupBox


    def createAccDataGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)
        self.label_acc_x.setText("0")
        self.label_acc_y.setText("0")
        self.label_acc_z.setText("0")
        layout.addWidget(self.label_acc_x)
        layout.addWidget(self.label_acc_y)
        layout.addWidget( self.label_acc_z)
        groupBox.setFixedWidth(150)
        groupBox.setTitle('线加速度')
        return groupBox    

    def createGyrDataGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)

        self.label_gyr_x.setText("0")
        self.label_gyr_y.setText("0")
        self.label_gyr_z.setText("0")
        layout.addWidget(self.label_gyr_x)
        layout.addWidget(self.label_gyr_y)
        layout.addWidget( self.label_gyr_z)

        groupBox.setFixedWidth(150)
        groupBox.setTitle('角速度')
        return groupBox    

    def createMagDataGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)

        self.label_mag_x.setText("0")
        self.label_mag_y.setText("0")
        self.label_mag_z.setText("0")
        layout.addWidget(self.label_mag_x)
        layout.addWidget(self.label_mag_y)
        layout.addWidget( self.label_mag_z)

        groupBox.setFixedWidth(150)
        groupBox.setTitle('磁力计')
        return groupBox    

    def createAngleGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)

        self.label_ang_x.setText("0")
        self.label_ang_y.setText("0")
        self.label_ang_z.setText("0")
        layout.addWidget(self.label_ang_x)
        layout.addWidget(self.label_ang_y)
        layout.addWidget( self.label_ang_z)

        groupBox.setFixedWidth(150)
        groupBox.setTitle('姿态角')
        return groupBox 


    def createFirstExclusiveGroup(self):
        groupBox = QtWidgets.QGroupBox("Exclusive Radio Buttons")
        radio1 = QtWidgets.QRadioButton("&Radio button 1")## &显示为下划线
        radio2 = QtWidgets.QRadioButton("R&adio button 2")
        radio3 = QtWidgets.QRadioButton("Ra&dio button 3")
        
        radio1.setChecked(True)
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(radio1)
        vbox.addWidget(radio2)
        vbox.addWidget(radio3)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox

    def createConnGroups(self):
        groupBox = QtWidgets.QGroupBox()
        button1 = QtWidgets.QPushButton("连接")
        button2 = QtWidgets.QPushButton("断开")
        label1 = QtWidgets.QLabel()
        label1.setFixedWidth(25)
        label1.setFixedHeight(25)
        label1.setAlignment(QtCore.Qt.AlignCenter)
        label1.setText("")
        label1.setStyleSheet("background-color:red; border-width:1px; border-radius: 12px;")
        lineEdit1 = QtWidgets.QLineEdit()
        lineEdit1.setFixedWidth(100)
        lineEdit1.setText("192.168.2.1")

        lineEdit2 = QtWidgets.QLineEdit()
        lineEdit2.setFixedWidth(60)
        lineEdit2.setText("14560")

        vbox = QtWidgets.QHBoxLayout()
        vbox.addWidget(label1)
        vbox.addWidget(lineEdit1)
        vbox.addWidget(lineEdit2)
        vbox.addWidget(button1)
        vbox.addWidget(button2)
        groupBox.setLayout(vbox)
        groupBox.setMaximumHeight(80)
        return groupBox

    def getWidget(self):
        return self.widget