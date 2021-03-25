# from PySide2.QtCore import *
# from PySide2.QtGui import *
# from PySide2.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from OpenGL import GL,GLU,GLUT
from QCustomPlot2 import *

class DataPlot(QWidget):
    '''
    建立IMU，acc、gyro、attitude、magnetic的绘图界面
    '''
    def __init__(self,parent = None):
        super(DataPlot,self).__init__(parent)
        self.m_accPlot = QCustomPlot()
        self.m_gyrPlot = QCustomPlot()
        self.m_magPlot = QCustomPlot()
        self.m_anglePlot = QCustomPlot()
        self.m_tickCounter = 0.0
        ## 定时刷新
        self.m_timer = QTimer()
        self.m_timer.timeout.connect(self.timerCounter)
        self.m_timer.start(20)

        self.m_acc_x_Values = []
        self.m_acc_y_Values = []
        self.m_acc_z_Values = []
        self.m_gyr_x_Values = []
        self.m_gyr_y_Values = []
        self.m_gyr_z_Values = []
        self.m_mag_x_Values = []
        self.m_mag_y_Values = []
        self.m_mag_z_Values = []
        self.m_angle_x_Values = []
        self.m_angle_y_Values = []
        self.m_angle_z_Values = []
        self.m_timestamps = []
        self.dataLength = 800

    def createAccPlot(self):
        self.m_accPlot.yAxis.setRange(-25,25)
        self.m_accPlot.xAxis.setTickLength(1)

        self.m_accPlot.xAxis.setBasePen(QPen(Qt.white, 1))
        self.m_accPlot.yAxis.setBasePen(QPen(Qt.white, 1))
        self.m_accPlot.xAxis.setTickPen(QPen(Qt.white, 1))
        self.m_accPlot.yAxis.setTickPen(QPen(Qt.white, 1))
        self.m_accPlot.xAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_accPlot.yAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_accPlot.xAxis.setTickLabelColor(Qt.white)
        self.m_accPlot.yAxis.setTickLabelColor(Qt.white)
        self.m_accPlot.xAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_accPlot.yAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_accPlot.xAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_accPlot.yAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_accPlot.xAxis.grid().setSubGridVisible(True)
        self.m_accPlot.yAxis.grid().setSubGridVisible(True)
        self.m_accPlot.xAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_accPlot.yAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_accPlot.xAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))
        self.m_accPlot.yAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))

        plotGradient = QLinearGradient()
        plotGradient.setStart(0, 0)
        plotGradient.setFinalStop(0, 350)
        plotGradient.setColorAt(0, QColor(80, 80, 80))
        plotGradient.setColorAt(1, QColor(50, 50, 50))
        self.m_accPlot.setBackground(plotGradient)
        axisRectGradient = QLinearGradient()
        axisRectGradient.setStart(0, 0)
        axisRectGradient.setFinalStop(0, 350)
        axisRectGradient.setColorAt(0, QColor(80, 80, 80))
        axisRectGradient.setColorAt(1, QColor(30, 30, 30))
        self.m_accPlot.axisRect().setBackground(axisRectGradient)
        # acc x
        self.m_accPlot.addGraph()
        self.m_accPlot.graph(0).setData(self.m_acc_x_Values,self.m_timestamps)
        self.m_accPlot.graph(0).setPen(QPen(Qt.blue))
        # acc y
        self.m_accPlot.addGraph()
        self.m_accPlot.graph(1).setData(self.m_acc_y_Values,self.m_timestamps)
        self.m_accPlot.graph(1).setPen(QPen(Qt.red))
        # acc z
        self.m_accPlot.addGraph()
        self.m_accPlot.graph(2).setData(self.m_acc_z_Values,self.m_timestamps)
        self.m_accPlot.graph(2).setPen(QPen(Qt.green))

        self.m_accPlot.replot()
        return self.m_accPlot


    def createGyrPlot(self):
        self.m_gyrPlot.yAxis.setRange(-4,4)
        self.m_gyrPlot.xAxis.setTickLength(2)
        self.m_gyrPlot.xAxis.setBasePen(QPen(Qt.white, 1))
        self.m_gyrPlot.yAxis.setBasePen(QPen(Qt.white, 1))
        self.m_gyrPlot.xAxis.setTickPen(QPen(Qt.white, 1))
        self.m_gyrPlot.yAxis.setTickPen(QPen(Qt.white, 1))
        self.m_gyrPlot.xAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_gyrPlot.yAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_gyrPlot.xAxis.setTickLabelColor(Qt.white)
        self.m_gyrPlot.yAxis.setTickLabelColor(Qt.white)
        self.m_gyrPlot.xAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_gyrPlot.yAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_gyrPlot.xAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_gyrPlot.yAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_gyrPlot.xAxis.grid().setSubGridVisible(True)
        self.m_gyrPlot.yAxis.grid().setSubGridVisible(True)
        self.m_gyrPlot.xAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_gyrPlot.yAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_gyrPlot.xAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))
        self.m_gyrPlot.yAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))
        plotGradient = QLinearGradient()
        plotGradient.setStart(0, 0)
        plotGradient.setFinalStop(0, 350)
        plotGradient.setColorAt(0, QColor(80, 80, 80))
        plotGradient.setColorAt(1, QColor(50, 50, 50))
        self.m_gyrPlot.setBackground(plotGradient)
        axisRectGradient = QLinearGradient()
        axisRectGradient.setStart(0, 0)
        axisRectGradient.setFinalStop(0, 350)
        axisRectGradient.setColorAt(0, QColor(80, 80, 80))
        axisRectGradient.setColorAt(1, QColor(30, 30, 30))
        self.m_gyrPlot.axisRect().setBackground(axisRectGradient)

        self.m_gyrPlot.addGraph()
        self.m_gyrPlot.graph(0).setData(self.m_gyr_x_Values,self.m_timestamps)
        self.m_gyrPlot.graph(0).setPen(QPen(Qt.blue))

        self.m_gyrPlot.addGraph()
        self.m_gyrPlot.graph(1).setData(self.m_gyr_y_Values,self.m_timestamps)
        self.m_gyrPlot.graph(1).setPen(QPen(Qt.red))

        self.m_gyrPlot.addGraph()
        self.m_gyrPlot.graph(2).setData(self.m_gyr_z_Values,self.m_timestamps)
        self.m_gyrPlot.graph(2).setPen(QPen(Qt.green))

        self.m_gyrPlot.replot()
        return self.m_gyrPlot

    def createMagPlot(self):
        self.m_magPlot.yAxis.setRange(-600,600)

        # set some pens, brushes and backgrounds:
        self.m_magPlot.xAxis.setBasePen(QPen(Qt.white, 1))
        self.m_magPlot.yAxis.setBasePen(QPen(Qt.white, 1))
        self.m_magPlot.xAxis.setTickPen(QPen(Qt.white, 1))
        self.m_magPlot.yAxis.setTickPen(QPen(Qt.white, 1))
        self.m_magPlot.xAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_magPlot.yAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_magPlot.xAxis.setTickLabelColor(Qt.white)
        self.m_magPlot.yAxis.setTickLabelColor(Qt.white)
        self.m_magPlot.xAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_magPlot.yAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_magPlot.xAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_magPlot.yAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_magPlot.xAxis.grid().setSubGridVisible(True)
        self.m_magPlot.yAxis.grid().setSubGridVisible(True)
        self.m_magPlot.xAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_magPlot.yAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_magPlot.xAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))
        self.m_magPlot.yAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))
        plotGradient = QLinearGradient()
        plotGradient.setStart(0, 0)
        plotGradient.setFinalStop(0, 350)
        plotGradient.setColorAt(0, QColor(80, 80, 80))
        plotGradient.setColorAt(1, QColor(50, 50, 50))
        self.m_magPlot.setBackground(plotGradient)
        axisRectGradient = QLinearGradient()
        axisRectGradient.setStart(0, 0)
        axisRectGradient.setFinalStop(0, 350)
        axisRectGradient.setColorAt(0, QColor(80, 80, 80))
        axisRectGradient.setColorAt(1, QColor(30, 30, 30))
        self.m_magPlot.axisRect().setBackground(axisRectGradient)

        # create graphs
        self.m_magPlot.addGraph()
        self.m_magPlot.graph(0).setData(self.m_mag_x_Values,self.m_timestamps)
        self.m_magPlot.graph(0).setPen(QPen(Qt.blue))

        # mag y graph
        self.m_magPlot.addGraph()
        self.m_magPlot.graph(1).setData(self.m_mag_y_Values,self.m_timestamps)
        self.m_magPlot.graph(1).setPen(QPen(Qt.red))

        # mag z graph
        self.m_magPlot.addGraph()
        self.m_magPlot.graph(2).setData(self.m_mag_z_Values,self.m_timestamps)
        self.m_magPlot.graph(2).setPen(QPen(Qt.green))

        self.m_magPlot.replot()
        return self.m_magPlot

    def createAnglePlot(self):
        self.m_anglePlot.yAxis.setRange(-180,180)

        # set some pens, brushes and backgrounds:
        self.m_anglePlot.xAxis.setBasePen(QPen(Qt.white, 1))
        self.m_anglePlot.yAxis.setBasePen(QPen(Qt.white, 1))
        self.m_anglePlot.xAxis.setTickPen(QPen(Qt.white, 1))
        self.m_anglePlot.yAxis.setTickPen(QPen(Qt.white, 1))
        self.m_anglePlot.xAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_anglePlot.yAxis.setSubTickPen(QPen(Qt.white, 1))
        self.m_anglePlot.xAxis.setTickLabelColor(Qt.white)
        self.m_anglePlot.yAxis.setTickLabelColor(Qt.white)
        self.m_anglePlot.xAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_anglePlot.yAxis.grid().setPen(QPen(QColor(140, 140, 140), 1, Qt.DotLine))
        self.m_anglePlot.xAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_anglePlot.yAxis.grid().setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt.DotLine))
        self.m_anglePlot.xAxis.grid().setSubGridVisible(True)
        self.m_anglePlot.yAxis.grid().setSubGridVisible(True)
        self.m_anglePlot.xAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_anglePlot.yAxis.grid().setZeroLinePen(QPen(Qt.NoPen))
        self.m_anglePlot.xAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))
        self.m_anglePlot.yAxis.setUpperEnding(QCPLineEnding(QCPLineEnding.esSpikeArrow))
        plotGradient = QLinearGradient()
        plotGradient.setStart(0, 0)
        plotGradient.setFinalStop(0, 350)
        plotGradient.setColorAt(0, QColor(80, 80, 80))
        plotGradient.setColorAt(1, QColor(50, 50, 50))
        self.m_anglePlot.setBackground(plotGradient)
        axisRectGradient = QLinearGradient()
        axisRectGradient.setStart(0, 0)
        axisRectGradient.setFinalStop(0, 350)
        axisRectGradient.setColorAt(0, QColor(80, 80, 80))
        axisRectGradient.setColorAt(1, QColor(30, 30, 30))
        self.m_anglePlot.axisRect().setBackground(axisRectGradient)

        # create graphs
        self.m_anglePlot.addGraph()
        self.m_anglePlot.graph(0).setData(self.m_angle_x_Values,self.m_timestamps)
        self.m_anglePlot.graph(0).setPen(QPen(Qt.blue))

        # angle y graph
        self.m_anglePlot.addGraph()
        self.m_anglePlot.graph(1).setData(self.m_angle_y_Values,self.m_timestamps)
        self.m_anglePlot.graph(1).setPen(QPen(Qt.red))

        # angle z graph
        self.m_anglePlot.addGraph()
        self.m_anglePlot.graph(2).setData(self.m_angle_z_Values,self.m_timestamps)
        self.m_anglePlot.graph(2).setPen(QPen(Qt.green))

        self.m_anglePlot.replot()
        return self.m_anglePlot

    def timerCounter(self):
        self.m_tickCounter += 0.05
        self.m_accPlot.xAxis.setRange(self.m_tickCounter-10,self.m_tickCounter +2)
        self.m_gyrPlot.xAxis.setRange(self.m_tickCounter-10,self.m_tickCounter +2)
        self.m_magPlot.xAxis.setRange(self.m_tickCounter-10,self.m_tickCounter +2)
        self.m_anglePlot.xAxis.setRange(self.m_tickCounter-10,self.m_tickCounter +2)

        self.m_accPlot.replot()
        self.m_gyrPlot.replot()
        self.m_magPlot.replot()
        self.m_anglePlot.replot()


    def updateSensorData(self,accData,gyrData,magData):
        # save timestamp
        self.m_timestamps.append(self.m_tickCounter)

        # acc
        self.m_acc_x_Values.append(accData[0])
        self.m_acc_y_Values.append(accData[1])
        self.m_acc_z_Values.append(accData[2])
        self.m_accPlot.graph(0).setData(self.m_timestamps,self.m_acc_x_Values)
        self.m_accPlot.graph(1).setData(self.m_timestamps,self.m_acc_y_Values)
        self.m_accPlot.graph(2).setData(self.m_timestamps,self.m_acc_z_Values)

        #gyr
        self.m_gyr_x_Values.append(gyrData[0])
        self.m_gyr_y_Values.append(gyrData[1])
        self.m_gyr_z_Values.append(gyrData[2])
        self.m_gyrPlot.graph(0).setData(self.m_timestamps,self.m_gyr_x_Values)
        self.m_gyrPlot.graph(1).setData(self.m_timestamps,self.m_gyr_y_Values)
        self.m_gyrPlot.graph(2).setData(self.m_timestamps,self.m_gyr_z_Values)

        # mag
        self.m_mag_x_Values.append(magData[0])
        self.m_mag_y_Values.append(magData[1])
        self.m_mag_z_Values.append(magData[2])
        self.m_magPlot.graph(0).setData(self.m_timestamps,self.m_mag_x_Values)
        self.m_magPlot.graph(1).setData(self.m_timestamps,self.m_mag_y_Values)
        self.m_magPlot.graph(2).setData(self.m_timestamps,self.m_mag_z_Values)

        # change vector leght
        if(len(self.m_timestamps) > self.dataLength):
            del(self.m_timestamps[0])
        
        if(len(self.m_acc_x_Values) > self.dataLength):
            del(self.m_acc_x_Values[0])
        
        if(len(self.m_acc_y_Values) > self.dataLength):
            del(self.m_acc_y_Values[0])
        
        if(len(self.m_acc_z_Values) > self.dataLength):
            del(self.m_acc_z_Values[0])
        
        if(len(self.m_gyr_x_Values) > self.dataLength):
            del(self.m_gyr_x_Values[0])
        
        if(len(self.m_gyr_y_Values) > self.dataLength):
            del(self.m_gyr_y_Values[0])
        
        if(len(self.m_gyr_z_Values) > self.dataLength):
            del(self.m_gyr_z_Values[0])
        
        if(len(self.m_mag_x_Values) > self.dataLength):
            del(self.m_mag_x_Values[0])
        
        if(len(self.m_mag_y_Values) > self.dataLength):
            del(self.m_mag_y_Values[0])
        
        if(len(self.m_mag_z_Values) > self.dataLength):
            del(self.m_mag_z_Values[0])
        

    def updateAngleData(self, angleData):
        # angle
        self.m_angle_x_Values.append(angleData[0])
        self.m_angle_y_Values.append(angleData[1])
        self.m_angle_z_Values.append(angleData[2])
        self.m_anglePlot.graph(0).setData(self.m_timestamps,self.m_angle_x_Values)
        self.m_anglePlot.graph(1).setData(self.m_timestamps,self.m_angle_y_Values)
        self.m_anglePlot.graph(2).setData(self.m_timestamps,self.m_angle_z_Values)

        if(self.m_angle_x_Values.count() > self.dataLength):
            del self.m_angle_x_Values[0]
        
        if(self.m_angle_y_Values.count() > self.dataLength):
            del self.m_angle_y_Values[0]

        if(self.m_angle_z_Values.count() > self.dataLength):
            del self.m_angle_z_Values[0]
        