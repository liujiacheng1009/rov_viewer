from PyQt5 import QtGui, QtCore, QtWidgets,uic
import sys

class Display(QtWidgets.QMainWindow):
    """Manage the gui
    Listen topics to display information
    Publish parameters for controllers and target regularly in the display method
    """
    def _depth_param_clicked(self):
        pass

    def _heading_param_clicked(self):
        pass


    def _velocity_param_clicked(self):
        pass
    def _target_param_clicked(self):
        pass

    def _pwm_max_clicked(self):
        pass
    def _activate_depth_ctrl_checked(self):
        pass

    def _activate_headind_ctrl_checked(self):
        pass

    def _activate_velocity_ctrl_checked(self):
        pass

    def _enable_automatic_control_checked(self):
        pass

    def _enable_publish_target(self):
        pass
        
    def _record_depth_clicked(self):
        pass

    def _record_heading_clicked(self):
        pass

    def _record_velocity_clicked(self):
        pass
 
    def _record_all_clicked(self):
        pass

    def __init__(self):
        super(Display, self).__init__() 
        
        uic.loadUi("bluerov.ui", self)
        self.pushButton_send_parameters_heading.clicked.connect(self._heading_param_clicked)
        self.pushButton_send_parameters_velocity.clicked.connect(self._velocity_param_clicked)
        self.pushButton_send_parameters_depth.clicked.connect(self._depth_param_clicked)
        self.pushButton_send_parameters_target.clicked.connect(self._target_param_clicked)
        self.pushButton_send_pwm_max.clicked.connect(self._pwm_max_clicked)
        self.pushButton_record_depth.clicked.connect(self._record_depth_clicked)
        self.pushButton_record_heading.clicked.connect(self._record_heading_clicked)
        self.pushButton_record_velocity.clicked.connect(self._record_velocity_clicked)
        self.pushButton_record_all.clicked.connect(self._record_all_clicked)
        self.checkBox_activate_depth_controller.clicked.connect(self._activate_depth_ctrl_checked)
        self.checkBox_activate_heading_controller.clicked.connect(self._activate_headind_ctrl_checked)
        self.checkBox_activate_velocity_controller.clicked.connect(self._activate_velocity_ctrl_checked)
        self.checkBox_enable_automatic_control.clicked.connect(self._enable_automatic_control_checked)
        #self.checkBox_forbid_publish_target.clicked.connect(self._forbid_publish_target)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.display)
        self.timer.start(250)
        


    def _state_callback(self,msg):
        pass

    def _battery_callback(self, msg):
        pass

    def _callback_depth(self,msg):
        pass
    def _callback_heading(self,msg):
        pass

    def _callback_velocity(self,msg):
        pass

    def _callback_joy(self,msg):
        pass
    def _settings_depth_ctrl_callback(self,msg):
        pass

    def _settings_heading_ctrl_callback(self,msg):
        pass

    def _settings_velocity_ctrl_callback(self,msg):
        pass
    
    def _settings_target_callback(self, msg):
        pass

    def _bar30_callback(self,msg):
        pass

    def _callback_attitude(self, msg):
        pass

    def display(self):
        pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Display()
    window.show()
    app.exec_()