from PyQt5 import QtGui, QtCore, QtWidgets,uic
import sys

class Display(QtWidgets.QMainWindow):

    def __init__(self):
        super(Display, self).__init__() 
        
        uic.loadUi("bluerov.ui", self)


        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.display)
        self.timer.start(250)
        
    def display(self):
        pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Display()
    window.show()
    app.exec_()