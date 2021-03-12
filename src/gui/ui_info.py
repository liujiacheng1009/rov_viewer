import rospy 

class UI_INFO:
    '''
    设置主界面左侧的信息栏
    '''
    def __init__(self, ui):
        self.ui = ui 
        self.layout1 = ui.page_1

    def update(self):
        

