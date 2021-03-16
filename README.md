# bluerov2_ros_interface
bluerov2 的ros接口和界面

参考: https://github.com/cyrilcotsaftis/bluerov_ros_playground


实现功能：

1、将bluerov2的图像和IMU数据转换为ROS msg  

2、GUI显示数据、控制rov运动

![界面](https://github.com/realjc/rov_viewer/blob/main/doc/rov_viewer.gif)



外观要求：

- 有学校学院logo

功能要求：

- 能够读取深度、IMU、camera、灯光、相机云台信息
- 实现gamepad的替代
- 指定深度、速度、仰角保持
  

规划：
- mavlink.py px4信号的收发工作,应当是单例
- gstreamer.py camera信号的解析，单例

- gamepad.py gamepad的替代,通过按键获取控制信号，通过mavlink.py发送回px4 // 不是优先实现
- pid.py PID算法的实现,从界面获取kp、ki、kd和目标状态，输出pwm信号，通过mavlink.py发送回px4// 优先实现
    pid算法应当能够同时适应深度、速度、仰角目标。
    深度的测量值来自bar30，d = 100* bar_data[1]

现在能做的，获取imu、camera数据， 

- depth_controller.py 
