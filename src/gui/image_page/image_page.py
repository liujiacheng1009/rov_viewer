#!/usr/bin/env python3
'''
负责显示图像
'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import gui_rc
from image_page.yolo.yolo_wd import YOLOWd
from image_page.calibration.calib_wd import CalibWd

class ImagePage:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.image_label = QLabel()
        self.createWidget()

    def createWidget(self):
        self.tab_wd = QTabWidget()
        self.widget1 = self.createImgWidget()
        self.widget2 = CalibWd()
        self.yolo_wd = YOLOWd()
        self.tab_wd.addTab(self.widget1,"查看")
        self.tab_wd.addTab(self.widget2,"相机标定")
        self.tab_wd.addTab(self.yolo_wd,"目标检测")
        layout = QGridLayout()
        layout.addWidget(self.tab_wd)
        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox

    def createImgWidget(self):
        wd = QWidget()
        lt = QGridLayout()
        lt.addWidget(self.image_label)
        wd.setLayout(lt)
        return wd
        
