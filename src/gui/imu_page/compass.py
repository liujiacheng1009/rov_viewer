#from PySide2 import QtCore, QtGui,QtWidgets
from PyQt5 import QtCore 
from PyQt5 import QtWidgets 
from PyQt5 import QtGui 
from OpenGL import GL,GLU,GLUT

class Compass(QtWidgets.QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.m_angle = 0
    
    def width(self):
        return self.geometry().width()
    
    def height(self):
        return self.geometry().height()

    def paintEvent(self,event):
        p = QtGui.QPainter(self)
        p.setPen(QtCore.Qt.black)
        p.setBrush(QtCore.Qt.black)
        p.drawRect(0,0,self.width(),self.height())
        needleNL = [QtCore.QPoint(5,0),
            QtCore.QPoint(-5,0),
            QtCore.QPoint(0,-45)]
        needleNR = [QtCore.QPoint(-5,0),
            QtCore.QPoint(5,0),
            QtCore.QPoint(0,-45)]
        needleSL = [QtCore.QPoint(5,0),
            QtCore.QPoint(-5,0),
            QtCore.QPoint(0,45)]
        needleSR = [QtCore.QPoint(-5,0),
            QtCore.QPoint(5,0),
            QtCore.QPoint(0,45)]
        needleColorNL = QtGui.QColor(255, 0, 0)
        needleColorSR = QtGui.QColor(0,127,127,191)
        needleColorNR = QtGui.QColor(200, 0, 0)
        needleColorSL = QtGui.QColor(0, 100, 100, 191)
        painter = QtGui.QPainter(self)
        ##qMin从哪里引用
        side = min(self.width(),self.height())
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.translate(self.width()/2, self.height()/2)
        painter.scale(side/130.0, side/130.0)
        painter.save()
        painter.rotate(self.m_angle)

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(needleColorNL)
        painter.drawConvexPolygon(QtGui.QPolygon(needleNL))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(needleColorNR)
        painter.drawConvexPolygon(QtGui.QPolygon(needleNR))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(needleColorSL)
        painter.drawConvexPolygon(QtGui.QPolygon(needleSL))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(needleColorSR)
        painter.drawConvexPolygon(QtGui.QPolygon(needleSR))

        painter.restore()

        pen = QtGui.QPen()
        pen.setWidth(0.5)
        pen.setBrush(QtCore.Qt.green)
        pen.setStyle(QtCore.Qt.SolidLine)
        painter.setPen(pen)
        painter.drawText(-5,-52, "N")
        painter.drawText(-80,-50,str(-int(self.m_angle-360))+"°" )
        for i in range(360):
            painter.drawLine(50,0,48,0)
            painter.rotate(1.0)
        for i in range(120):
            painter.drawLine(50,0,46,0)
            painter.rotate(3.0)
        pen.setWidth(0.6)
        painter.setPen(pen)
        for i in range(8):
            painter.drawLine(50,0,43,0)
            painter.rotate(45.0)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawEllipse(QtCore.QPoint(0,0),50,50)
        
    def updateAngle(self,angle):
        self.m_angle = angle
        self.update()
