#from PySide2 import QtCore, QtGui,QtWidgets
from PyQt5 import QtCore 
from PyQt5 import QtWidgets 
from PyQt5 import QtGui 
#from conn import ConnWidget
import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *

from imu_page.visualization import Visualization
from imu_page.compass import Compass
from imu_page.dataplot import DataPlot

class IMUPage:
    def __init__(self, parent = None):
        self.widget = QtWidgets.QWidget()
        # self.connGroup = ConnWidget()
        self.visualization = Visualization()
        self.compass = Compass()
        self.dataplot = DataPlot()
        grid = QtWidgets.QGridLayout()
        #grid.addWidget(self.createFirstExclusiveGroup(), 0, 0)
        grid.addWidget(self.createConnGroups(),0,0,1,2)
        #grid.addWidget(self.connGroup.groupBox,2,0,1,2)
        grid.addWidget(self.createVisualGroups(), 1,0,4,1)
        grid.addWidget(self.createAccDataGroupbox(),1,1)
        grid.addWidget(self.createGyrDataGroupbox(),2,1)
        grid.addWidget(self.createMagDataGroupbox(),3,1)
        grid.addWidget(self.createAngleGroupbox(),4,1)

        self.widget.setLayout(grid)
        # self.setWindowTitle("Group Box")
        # self.resize(640, 480)

    def createVisualGroups(self):
        groupBox = QtWidgets.QGroupBox()
        tabWidget = QtWidgets.QTabWidget()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)
        layout.addWidget(tabWidget)

        widget1 = self.visualization
        widget2 = self.compass
        widget3 = self.dataplot.createAnglePlot()
        widget4 = self.dataplot.createAccPlot()
        widget5 = self.dataplot.createGyrPlot()
        widget6 = self.dataplot.createMagPlot()
        tabWidget.addTab(widget1,"3D")
        tabWidget.addTab(widget2,"Compass")
        tabWidget.addTab(widget3,"Angles")
        tabWidget.addTab(widget4,"Acc")
        tabWidget.addTab(widget5,"Gyr")
        tabWidget.addTab(widget6,"Mag")

        return groupBox


    def createAccDataGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)

        lableX = QtWidgets.QLabel()
        lableX.setText("0")
        lableY = QtWidgets.QLabel()
        lableY.setText("0")
        lableZ = QtWidgets.QLabel()
        lableZ.setText("0")

        layout.addWidget(lableX)
        layout.addWidget(lableY)
        layout.addWidget(lableZ)

        groupBox.setFixedWidth(150)
        return groupBox    

    def createGyrDataGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)

        lableX = QtWidgets.QLabel()
        lableX.setText("0")
        lableY = QtWidgets.QLabel()
        lableY.setText("0")
        lableZ = QtWidgets.QLabel()
        lableZ.setText("0")

        layout.addWidget(lableX)
        layout.addWidget(lableY)
        layout.addWidget(lableZ)

        groupBox.setFixedWidth(150)
        return groupBox    

    def createMagDataGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)

        lableX = QtWidgets.QLabel()
        lableX.setText("0")
        lableY = QtWidgets.QLabel()
        lableY.setText("0")
        lableZ = QtWidgets.QLabel()
        lableZ.setText("0")

        layout.addWidget(lableX)
        layout.addWidget(lableY)
        layout.addWidget(lableZ)

        groupBox.setFixedWidth(150)
        return groupBox    

    def createAngleGroupbox(self):
        groupBox = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout()
        groupBox.setLayout(layout)

        lableX = QtWidgets.QLabel()
        lableX.setText("0")
        lableY = QtWidgets.QLabel()
        lableY.setText("0")
        lableZ = QtWidgets.QLabel()
        lableZ.setText("0")

        layout.addWidget(lableX)
        layout.addWidget(lableY)
        layout.addWidget(lableZ)

        groupBox.setFixedWidth(150)
        return groupBox    


    def createFirstExclusiveGroup(self):
        groupBox = QtWidgets.QGroupBox("Exclusive Radio Buttons")
        radio1 = QtWidgets.QRadioButton("&Radio button 1")## &显示为下划线
        radio2 = QtWidgets.QRadioButton("R&adio button 2")
        radio3 = QtWidgets.QRadioButton("Ra&dio button 3")
        
        radio1.setChecked(True)
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(radio1)
        vbox.addWidget(radio2)
        vbox.addWidget(radio3)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox

    def createConnGroups(self):
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

    def getWidget(self):
        return self.widget