#!/usr/bin/env python3
'''
负责绘制首页

'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import gui_rc

class BeginPage:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.createWidget()

    def createWidget(self):
        button1 = QPushButton()
        pixmap = QPixmap(":/icon/icon/conn.ico")
        ButtonIcon = QIcon(pixmap)
        button1.setIcon(ButtonIcon)
        button1.setIconSize(pixmap.rect().size())
                
        # button1.resize(64, 64) 
        # button1.setStyleSheet("QPushButton{border-image: url(:/icon/icon/conn.ico)}"
        #                           "QPushButton:hover{border-image: url(:/icon/icon/conn.ico)}" 
        #                           "QPushButton:pressed{border-image: url(:/icon/icon/conn.ico)}")
        button2 = QPushButton()
        button2.resize(64, 64) 
        button2.setStyleSheet("QPushButton{border-image: url(:/icon/icon/lock.ico)}"
                                  "QPushButton:hover{border-image: url(:/icon/icon/lock.ico)}" 
                                  "QPushButton:pressed{border-image: url(:/icon/icon/lock.ico)}")
        button3 = QPushButton()
        button3.resize(64, 64) 
        button3.setStyleSheet("QPushButton{border-image: url(:/icon/icon/imu.ico)}"
                                  "QPushButton:hover{border-image: url(:/icon/icon/imu.ico)}" 
                                  "QPushButton:pressed{border-image: url(:/icon/icon/imu.ico)}")
        button4 = QPushButton()
        button4.resize(64, 64) 
        button4.setStyleSheet("QPushButton{border-image: url(:/icon/icon/camera.png)}"
                                  "QPushButton:hover{border-image: url(:/icon/icon/camera.png)}" 
                                  "QPushButton:pressed{border-image: url(:/icon/icon/camera.png)}")
        button5 = QPushButton()
        button5.resize(64, 64) 
        button5.setStyleSheet("QPushButton{border-image: url(:/icon/icon/gamepad1.ico)}"
                                  "QPushButton:hover{border-image: url(:/icon/icon/gamepad1.ico)}" 
                                  "QPushButton:pressed{border-image: url(:/icon/icon/gamepad1.ico)}")
        button6 = QPushButton()
        button6.resize(64, 64) 
        button6.setStyleSheet("QPushButton{border-image: url(:/icon/icon/depth.png)}"
                                  "QPushButton:hover{border-image: url(:/icon/icon/depth.png)}" 
                                  "QPushButton:pressed{border-image: url(:/icon/icon/depth.png)}")
        button7 = QPushButton()
        button7.resize(64, 64) 
        button7.setStyleSheet("QPushButton{border-image: url(:/icon/icon/yaw.png)}"
                                  "QPushButton:hover{border-image: url(:/icon/icon/yaw.png)}" 
                                  "QPushButton:pressed{border-image: url(:/icon/icon/yaw.png)}")
        button8 = QPushButton()
        button8.resize(64, 64) 
        button8.setStyleSheet("QPushButton{border-image: url(:/icon/icon/velo.ico)}"
                                  "QPushButton:hover{border-image: url(:/icon/icon/velo.ico)}" 
                                  "QPushButton:pressed{border-image: url(:/icon/icon/velo.ico)}")

        layout = QGridLayout()
        layout.addWidget(button1,0,0)
        layout.addWidget(button2,0,1)
        layout.addWidget(button3,0,2)
        layout.addWidget(button4,0,3)
        layout.addWidget(button5,1,0)
        layout.addWidget(button6,1,1)
        layout.addWidget(button7,1,2)
        layout.addWidget(button8,1,3)

        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox