from PySide2 import QtCore, QtGui,QtWidgets

class ConnWidget():
    def __init__(self):
        self.groupBox = self.creatConnGroup()

    def creatConnGroup(self):
        groupBox = QtWidgets.QGroupBox()
        button1 = QtWidgets.QPushButton("连接")
        button2 = QtWidgets.QPushButton("断开")
        label1 = QtWidgets.QLabel()
        label1.setFixedWidth(25)
        label1.setFixedHeight(25)
        label1.setAlignment(QtCore.Qt.AlignCenter)
        label1.setText("")
        label1.setStyleSheet("background-color:red; border-width:1px; border-radius: 12px;")
        lineEdit1 = QtWidgets.QLineEdit()
        lineEdit1.setFixedWidth(100)
        lineEdit1.setText("192.168.2.1")

        lineEdit2 = QtWidgets.QLineEdit()
        lineEdit2.setFixedWidth(60)
        lineEdit2.setText("14560")

        vbox = QtWidgets.QHBoxLayout()
        vbox.addWidget(label1)
        vbox.addWidget(lineEdit1)
        vbox.addWidget(lineEdit2)
        vbox.addWidget(button1)
        vbox.addWidget(button2)
        groupBox.setLayout(vbox)
        groupBox.setMaximumHeight(80)
        return groupBox