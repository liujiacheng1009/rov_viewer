#!/usr/bin/env python3
'''
负责PID控制
'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import gui_rc


class CtrWnd(QGroupBox):
    def __init__(self,parent=None):
        super(CtrWnd,self).__init__(parent)
        layout = QVBoxLayout()
        gb1 = self.createGB1()
        gb2 = self.createGB2()
        gb3 = self.createGB3()
        gb4 = self.createGB4()
        layout.addWidget(gb1)
        layout.addWidget(gb2)
        layout.addWidget(gb3)
        layout.addWidget(gb4)
        self.setLayout(layout)

    def createGB1(self):
        gb = QGroupBox()
        lt = QGridLayout()
        label1 = QLabel("目标值：")
        label2 = QLabel("测量值：")
        label3 = QLineEdit()
        label4 = QLineEdit()
        lt.addWidget(label1,0,0)
        lt.addWidget(label2,1,0)
        lt.addWidget(label3,0,1)
        lt.addWidget(label4,1,1)
        gb.setLayout(lt)
        return gb 

    def createGB2(self):
        gb = QGroupBox()
        lt = QGridLayout()
        label1 = QLabel("最大PWM: ")
        label2 = QLabel("最小PWM: ")
        label3 = QLineEdit()
        label4 = QLineEdit()
        lt.addWidget(label1,0,0)
        lt.addWidget(label2,1,0)
        lt.addWidget(label3,0,1)
        lt.addWidget(label4,1,1)
        gb.setLayout(lt)
        return gb

    def createGB3(self):
        gb = QGroupBox()
        lt = QGridLayout()
        label1 = QLabel("参数P: ")
        label2 = QLabel("参数I: ")
        label3 = QLabel("参数D: ")
        label4 = QLineEdit()
        label5 = QLineEdit()
        label6 = QLineEdit()
        lt.addWidget(label1,0,0)
        lt.addWidget(label2,1,0)
        lt.addWidget(label3,2,0)
        lt.addWidget(label4,0,1)
        lt.addWidget(label5,1,1)
        lt.addWidget(label6,2,1)
        gb.setLayout(lt)
        return gb 

    def createGB4(self):
        gb = QGroupBox()
        lt = QHBoxLayout()
        pb1 = QPushButton("Exit")
        pb2 = QPushButton("Run")
        lt.addWidget(pb1)
        lt.addWidget(pb2)
        gb.setLayout(lt)
        return gb



class PIDCtrPage:
    def __init__(self):
        self.groupBox = QGroupBox()
        self.createWidget()

    def createWidget(self):
        layout = QHBoxLayout()
        widget1 = CtrWnd()
        widget1.setTitle("定深")
        widget2 = CtrWnd()
        widget2.setTitle("定向")
        widget3 = CtrWnd()
        widget3.setTitle("定速")
        layout.addWidget(widget1)
        layout.addWidget(widget2)
        layout.addWidget(widget3)
        self.groupBox.setLayout(layout)

    def getWidget(self):
        return self.groupBox
