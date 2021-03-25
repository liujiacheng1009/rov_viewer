#!/usr/bin/env python3
'''
负责绘制首页

'''
import sys 
# import os 
# curr_dir = os.getcwd()
sys.path.append('/home/bluerov/Downloads/catkin_ws/src/rov_viewer/src/bluerov')
# print(curr_dir)

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from bluerov import BlueRov

import gui_rc
import icon_rc

class BeginPage:
    def __init__(self,gui):
        self.gui = gui
        self.groupBox = QGroupBox()
        self.createWidget()

    def createWidget(self):
        bt_size = QSize(100,100)
        icons = [ "lock1.png", "imu3.png", "camera.png",
                    "gamepad1.png","depth.png", "yaw.png", "speed.png","navigation.png"]
        bt_path = ":/icons/"

        labels = ["锁定/解锁", "IMU","相机","手动控制","定深", "定向","定速","自主导航"]
        buttons = []
        for icon in icons:
            bt = QPushButton()
            pixmap = QPixmap(bt_path+icon).scaled(bt_size)
            bt.setIcon(QIcon(pixmap))
            bt.setIconSize(bt_size)
            bt.setFixedSize(bt_size)
            buttons.append(bt)
        layout = QGridLayout()
        for i in range(len(icons)):
            buttons[i].clicked.connect(lambda state, x=i: self.button_pushed(x))
            layout.addWidget(buttons[i],int(i/4)*2,int(i%4),alignment=Qt.AlignVCenter,)
            label = QLabel()
            label.setText(labels[i])
            label.setAlignment(Qt.AlignTop)
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label,int(i/4)*2+1,int(i%4),alignment=Qt.AlignTop)
        self.groupBox.setLayout(layout)

    def createConnGroups(self):
        groupBox = QGroupBox()
        label1 = QLabel()
        label1.setFixedWidth(25)
        label1.setFixedHeight(25)
        label1.setAlignment(Qt.AlignCenter)
        label1.setText("")
        label1.setStyleSheet("background-color:red; border-width:1px; border-radius: 12px;")
        lineEdit1 = QLineEdit()
        lineEdit1.setFixedWidth(100)
        lineEdit1.setText("192.168.2.1")

        lineEdit2 = QLineEdit()
        lineEdit2.setFixedWidth(60)
        lineEdit2.setText("14560")

        vbox = QHBoxLayout()
        vbox.addWidget(label1)
        vbox.addWidget(lineEdit1)
        vbox.addWidget(lineEdit2)
        groupBox.setLayout(vbox)
        groupBox.setMaximumHeight(80)
        return groupBox


    def getWidget(self):
        return self.groupBox

    def button_pushed(self,i):
        if i<=3:
            self.gui.tabWidget.setCurrentIndex(i)
            return
        if i<7:
            self.gui.tabWidget.setCurrentIndex(4)
            return
        if i==7:
            self.gui.tabWidget.setCurrentIndex(5)
            return


class ConnWidget:
    def __init__(self,parent):
        self.parent = parent
        self.lineEdit1 = QLineEdit()
        self.lineEdit1.setFixedWidth(100)
        self.lineEdit1.setText("192.168.2.1")

        self.lineEdit2 = QLineEdit()
        self.lineEdit2.setFixedWidth(60)
        self.lineEdit2.setText("14560")

    def creatConnGroup(self):
        groupBox = QGroupBox()
        button1 = QPushButton("连接")
        #button1.clicked.connect(self.bt1_callback)
        button2 = QPushButton("断开")
        #button2.clicked.connect(self.bt2_callback)
        label1 = QLabel()
        label1.setFixedWidth(25)
        label1.setFixedHeight(25)
        label1.setAlignment(Qt.AlignCenter)
        label1.setText("")
        label1.setStyleSheet("background-color:red; border-width:1px; border-radius: 12px;")


        vbox = QHBoxLayout()
        vbox.addWidget(label1)
        vbox.addWidget(self.lineEdit1)
        vbox.addWidget(self.lineEdit2)
        vbox.addWidget(button1)
        vbox.addWidget(button2)
        groupBox.setLayout(vbox)
        groupBox.setMaximumHeight(80)
        return groupBox

    
    def bt1_callback(self):
        ip = self.lineEdit1.text()
        port = self.lineEdit2.text()
        device = 'udp:'+ip+':'+port
        if(self.parent.bluerov is not None):
            return
        try:
            self.parent.bluerov = BlueRov(device= device)
            print(device)
        except:
            print("Unexpected error:", sys.exc_info()[0])

    
    def bt2_callback(self):
        if(self.parent.bluerov is not None):
            del self.parent.bluerov
            self.parent.bluerov = None


class ROVInfo:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.createWidget()
        self.groupBox.setFixedWidth(200)

    def createWidget(self):
        layout = QVBoxLayout()
        ##上方的rov图片
        label1 = QLabel()
       # label1.setGeometry(0,0,200,200)
        image = QImage()
        image.load(":/img/img/bluerov2.png")
        label1.setPixmap(QPixmap(image.scaled(200,200,Qt.KeepAspectRatio)))
        layout.addWidget(label1)
        ## rov基本信息
        label2 = QLabel()
        label2.setText("设备介绍:")
        label2.setStyleSheet("font-weight: bold"); 
        label2.setAlignment(Qt.AlignLeft)
        layout.addWidget(label2)
        label3 = QLabel()
        label3.setWordWrap(True)
        label3.setAlignment(Qt.AlignTop)
        label3.setText("BlueROV Heavy是由蓝色机器人公司的一款小型水下机器人,它搭载有8个推进器，能够实现6自由度的运动，并搭载有摄像头，方便观察水下环境。")
        layout.addWidget(label3)
        label4 = QLabel()
        label4.setText("软件介绍:")
        label4.setStyleSheet("font-weight: bold"); 
        layout.addWidget(label4)
        label5 = QLabel()
        label5.setWordWrap(True)
        label5.setAlignment(Qt.AlignTop)
        label5.setText("本软件旨在为后续开发提供ROS接口和交互界面。现初步实现了IMU、相机信息的显示、保存，手动/自动控制，相机标定，视觉检测等功能，项目仍在维护完善中。")
        layout.addWidget(label5)
        label6 = QLabel()
        img = QImage()
        img.load(":/img/img/scut2.png")
        label6.setPixmap(QPixmap(img.scaled(200,200,Qt.KeepAspectRatio)))
        layout.addWidget(label6)
        layout.setStretch(1,1)
        layout.setStretch(2,6)
        layout.setStretch(3,1)
        layout.setStretch(4,6)
        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox
