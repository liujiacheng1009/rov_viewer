
#!/usr/bin/env python3
'''
负责绘制左侧的ROV状态

'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import gui_rc

class ROVInfo:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.createWidget()

    def createWidget(self):
        layout = QVBoxLayout()
        ##上方的rov图片
        label1 = QLabel()
        label1.setGeometry(0,0,100,100)
        image = QImage()
        image.load(":/img/img/bluerov2.png")
        label1.setPixmap(QPixmap(image.scaled(200,200,Qt.KeepAspectRatio)))

        layout.addWidget(label1)
        ## rov基本信息
        label2 = QLabel()
        label2.setText("BlueROV2 Heavy ")
        layout.addWidget(label2)
        label3 = QLabel()
        label3.setText("推进器 8\n自由度 6 ")
        layout.addWidget(label3)
        ## rov的状态及传感器信息
        toolBox = QToolBox()
        label_tb1 = QLabel()
        toolBox.addItem(label_tb1, "状态")
        label_tb2 = QLabel()
        toolBox.addItem(label_tb2, "IMU")
        label_tb3 = QLabel()
        toolBox.addItem(label_tb3, "相机")
        label_tb4 = QLabel()
        toolBox.addItem(label_tb4, "其他")
        layout.addWidget(toolBox)

        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox
