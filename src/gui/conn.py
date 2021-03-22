from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

class ConnWidget():
    def __init__(self):
        pass

    def creatConnGroup(self):
        groupBox = QGroupBox()
        button1 = QPushButton("连接")
        button2 = QPushButton("断开")
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
        vbox.addWidget(button1)
        vbox.addWidget(button2)
        groupBox.setLayout(vbox)
        groupBox.setMaximumHeight(80)
        return groupBox
