#!/usr/bin/env python3
'''
负责手柄控制
'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import gui_rc

class ManualCtrPage:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.createWidget()

    def createWidget(self):
        layout = QGridLayout()
        label = QLabel()
        label.setScaledContents(True)
        layout.addWidget(label)
        ratio1 = QRadioButton("使用gamepad控制")
        ratio1.setChecked(True)
        layout.addWidget(ratio1)
        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox