这里主要是连接ROV
读取mavlink,gstreamer数据并发送出来

gst_reader.py: 将gst格式的图像数据流转化为opencv格式的图像
mav_bridge.py: 负责利用mavlink进行通信


bluerov_node.py 通过rostopic发布ROV信息(由gst、mavlink获取)，订阅topic控制ROV,
它将作为一个node独立运行.
BlueRov类继承自MAVBridge


发布的topic:
/BlueRov2/battery  发布电池信息
/BlueRov2/camera/image_raw 发布图像信息
/BlueRov2/state  ROV状态(自定义类型)， 包括锁定状态(arm)、rc1-rc6六个通道状态,light,camera(云台角度),mode
/BlueRov2/imu/data Imu格式的imu数据(旋转由四元数表示)
/BlueRov2/odometry 绝对轨迹 ，这里没有gps应该是无效的,未来做跟踪的话可能有用
/BlueRov2/bar30 深度传感器数据
/BlueRov2/imu/attitude rpy格式的IMU数据 

订阅的topic:
/BlueRov2/setpoint_velocity/cmd_del 设置角速度和线速度
/BlueRov2/servo{}/set_pwm 设置电机的pwm
/BlueRov2/rc_channel{}/set_pwm 设置RC通道的pwm
/BlueRov2/mode/set 设置模式
/BlueRov2/arm 设置锁定状态
/BlueROV2/manual_control 接收手柄控制信号

测试 test/test_gst_reader.py 
结果：
![gst测试](../../doc/test_gst_reader.png)

测试 test/test_mavlink_reader.py 
结果：


```bash
## 测试命令
# Set manual mode
$ rostopic pub -1 /BlueRov2/mode/set std_msgs/String "manual"
# Arm the vehicle
$ rostopic pub -1 /BlueRov2/Setting/arm std_msgs/Bool 1
# Set angular and linear speed
$ rostopic pub -r 4 /BlueRov2/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "{header: auto, twist: {linear: {x: 10.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
# Set MAIN OUT pwm value
$ rostopic pub -r 4 /BlueRov2/servo1/set_pwm std_msgs/UInt16  1500
# Set THROTTLE pwm value  
$ rostopic pub -r 4 /BlueRov2/rc_channel3/set_pwm std_msgs/UInt16  1900
# Visualize camera image
$ rosrun image_view image_view image:=/BlueRov2/camera/image_raw
# See ROV state
$ rostopic echo /BlueRov2/state
# Watch battery information
$ rostopic echo /BlueRov2/battery
# IMU information
$ rostopic echo /BlueRov2/imu/data
```

测试/BlueRov2/Setting/arm 发现可能要发送多次解锁命令才能成功
rc_channel3是有效的, 
servo1无反应,但是通过手柄本身也无法操作单个电机

gst_reader的c++ 版本可以参考：https://www.codeleading.com/article/14562585841/


pymavlink是通用的消息处理库,支持MAVLink1和MAVLink2版本
是一個用於小型無人機的通信協議
mavlink_connection 建立连接，可以有多种模式
wait_heartbeat 心跳信号包内包含飞机状态和控制信号

mission planner中的一些模式的解释
 自穩模式(Stabilize):自穩模式是 APM:Copter 最常用的手動飛行模式。
 定高模式(Alt Hold):保持穩定高度。
 停旋模式(Loiter):維持固定的位置、方向和高度。
 自返模式(RTL):自動返回初始多旋翼直升機起飛位置。
 自動模式(Auto):將按照飛行規劃中的任務設定進行飛行

bluerov.py 发送图像数据存在花屏的现象,可能要使用多线程解决

规划：
- mavlink.py px4信号的收发工作,应当是单例
- gstreamer.py camera信号的解析，单例

- gamepad.py gamepad的替代,通过按键获取控制信号，通过mavlink.py发送回px4 // 不是优先实现
- pid.py PID算法的实现,从界面获取kp、ki、kd和目标状态，输出pwm信号，通过mavlink.py发送回px4// 优先实现
    pid算法应当能够同时适应深度、速度、仰角目标。
    深度的测量值来自bar30，d = 100* bar_data[1]

现在能做的，获取imu、camera数据， 

- depth_controller.py 


先把ui改出来

这个ui看来还要搞一天，

qt 要装5.12版本的，与QCustomplot 存在依赖