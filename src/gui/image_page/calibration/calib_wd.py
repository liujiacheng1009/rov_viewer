#!/usr/bin/env python3
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys 
import signal
import rospy 

class CalibWd(QWidget):
    def __init__(self,parent = None):
        super(CalibWd,self).__init__(parent)
        layout = QGridLayout()
        label1 = QLabel("xx")
        label2 = QLabel("xx")
        pb1 = QPushButton("取消")
        pb2 = QPushButton("采样")
        gb = self.createCaliGB()
        layout.addWidget(label1,0,0,2,1)
        layout.addWidget(label2,0,1,1,2)
        layout.addWidget(pb1,1,1)
        layout.addWidget(pb2,1,2)
        layout.addWidget(gb,2,0,1,3)
        layout.setColumnStretch(0,5)
        layout.setColumnStretch(1,1)
        layout.setColumnStretch(2,1)
        layout.setRowStretch(0,5)
        layout.setRowStretch(1,1)
        layout.setRowStretch(2,3)
        self.setLayout(layout)
        self.resize(600,400)
        
    def createCaliGB(self):
        gb = QGroupBox()
        layout = QGridLayout()
        label1 = QLabel("相机模型")
        combb1 = QComboBox()
        label2 = QLabel("标定板类型")
        combb2 = QComboBox()
        label3 = QLabel("方块大小")
        edit1 = QLineEdit()

        pb1 = QPushButton("标定")
        pb2 = QPushButton("保存")

        label4 = QLabel("xx")
        layout.addWidget(label1,0,0)
        layout.addWidget(combb1,0,1)
        layout.addWidget(label2,0,2)
        layout.addWidget(combb2,0,3)
        layout.addWidget(label3,1,0)
        layout.addWidget(edit1,1,1)
        layout.addWidget(pb1,1,2)
        layout.addWidget(pb2,1,3)
        layout.addWidget(label4,0,4,2,2)
        layout.setColumnStretch(4,2)
        gb.setLayout(layout)
        return gb
