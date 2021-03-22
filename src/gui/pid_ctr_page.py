#!/usr/bin/env python3
'''
负责PID控制
'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import gui_rc


class PIDWidget:
    def __init__(self, mode):
        self.mode = mode
        self.widget = QGroupBox()
        self.init_widget()

    def getWidget(self):
        return self.widget

    def init_widget(self):
        layout = QGridLayout()
        label1 = QLabel(self.mode)
        label2 = QLineEdit()
        label3 = QLineEdit()
        




class PIDCtrPage:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.createWidget()

    def createWidget(self):
        layout = QHBoxLayout()
        widget1 = self.createDepthCtrWidget()
        widget2 = self.createYawCtrWidget()
        widget3 = self.createVeloCtrWidget()
        layout.addWidget(widget1)
        layout.addWidget(widget2)
        layout.addWidget(widget3)
        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox

    def createDepthCtrWidget(self):
        widget = QGroupBox()
        layout = QVBoxLayout()
        widget.setLayout(layout)
        return widget

    def createYawCtrWidget(self):
        widget = QGroupBox()
        layout = QVBoxLayout()
        widget.setLayout(layout)
        return widget

    def createVeloCtrWidget(self):
        widget = QGroupBox()
        layout = QVBoxLayout()
        widget.setLayout(layout)
        return widget        