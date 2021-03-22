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
        bt_size = QSize(100,100)
        icons = ["conn.ico", "lock.ico", "imu.ico", "camera.png",
                    "gamepad1.ico","depth.png", "yaw.png", "velo.ico"]
        bt_path = ":/icon/icon/"
        buttons = []
        for icon in icons:
            bt = QPushButton()
            pixmap = QPixmap(bt_path+icon).scaled(bt_size)
            bt.setIcon(QIcon(pixmap))
            bt.setIconSize(bt_size)
            bt.setFixedSize(bt_size)
            buttons.append(bt)
        layout = QGridLayout()
        for i in range(2):
            for j in range(4):
                layout.addWidget(buttons[i*4+j],i,j)
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