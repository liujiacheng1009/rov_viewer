#!/usr/bin/env python3
'''
负责显示图像
'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import gui_rc

class ImagePage:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.createWidget()

    def createWidget(self):
        label = QLabel()
        #label.setPixmap(QPixmap(":/img/img/underwater.jpg"))
        #label.resize(self.groupBox.geometry().width(),self.groupBox.geometry().height())
        label.setScaledContents(True)
        layout = QGridLayout()
        layout.addWidget(label)
        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox