#!/usr/bin/env python3
'''
实现了mavlink数据的解析
使用前需要修改对应ip和port, 默认为bluerov2 heavy的设置
'''


from pymavlink import mavutil

class MAVBridge(object):
    '''
    输入：传输协议udp,监听ip,端口(本机,需修改以太网设置)
    输出：所有通过mavlink传输的数据
    '''

    def __init__(self,device='udpin:192.168.2.1:14560',baudrate=115200):
        '''
        建立连接
        '''
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        self.conn.wait_heartbeat() 
        ## 设置监听信号的类型，和速率，1似乎是默认的
        self.conn.mav.request_data_stream_send(self.conn.target_system, self.conn.target_component,
                            mavutil.mavlink.MAV_DATA_STREAM_ALL,4, 1)
        ## 所有的得到的mavlink msg
        self.data = {}

    def get_data(self):
        """ 获取dict形式的mavlink数据

        Returns:
            TYPE: Dict
        """
        return self.data

    def update(self):
        """ 
        更新数据,保存于字典data
        """
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    
    def get_all_msgs(self):
        """ 
        将接收到的mavlink数据存入数组msgs
        """
        msgs = []
        while True:
            msg = self.conn.recv_match() ## 捕获符合要求的信号
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def print_data(self):
        """ 
        打印数据，用于测试
        """
        print(self.data)

    def set_mode(self, mode):
        """ 
        设置飞行模式，这里暂时用不上,默认是manual
        参见：http://ardupilot.org/copter/docs/flight-modes.html
        """
        mode = mode.upper()
        if mode not in self.conn.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.conn.mode_mapping().keys()))
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)


    def decode_mode(self, base_mode, custom_mode):
        """ 从心跳包中解析飞行模式和锁定状态

        输入:
            base_mode (TYPE): System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            custom_mode (TYPE): A bitfield for use for autopilot-specific flags.

        输出:
            [str, bool]: 飞行模式, 锁定状态
        """
        flight_mode = ""
        ## 一些标准模式
        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 'MANUAL'],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 'STABILIZE'],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 'GUIDED'],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, 'AUTO'],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, 'TEST']
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm

    def set_guided_mode(self):
        """ 设置引导模式，似乎被弃用了
        """
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)
    
    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        """ 发送长指令 
            通过一条长命令发送控制指令

        参数:
            command (mavlink command): Command
            params (list, optional): param1, param2, ..., param7
            confirmation (int, optional): Confirmation value
        """
        self.conn.mav.command_long_send(
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            command,                                # mavlink command
            confirmation,                           # confirmation
            params[0],                              # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6]
        )

    def set_position_target_local_ned(self, param=[]):
        """ 发送一个SET_POSITION_TARGET_LOCAL_NED 信号，设置目标位置?
        参数:
            param (list, optional): param1, param2, ..., param11
        """
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

        # Set mask
        mask = 0b0000000111111111
        for i, value in enumerate(param):
            if value is not None:
                mask -= 1<<i
            else:
                param[i] = 0.0

        self.conn.mav.set_position_target_local_ned_send(
            0,                                              # system time in milliseconds
            self.conn.target_system,                        # target system
            self.conn.target_component,                     # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # frame
            mask,                                           # mask
            param[0], param[1], param[2],                   # position x,y,z
            param[3], param[4], param[5],                   # velocity x,y,z
            param[6], param[7], param[8],                   # accel x,y,z
            param[9], param[10])                            # yaw, yaw rate

    def set_attitude_target(self, param=[]):
        """ 产生一个 SET_ATTITUDE_TARGET 信号，设置目标姿态
        参数:
            param (list, optional): param1, param2, ..., param7
        """
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.conn.mav.set_attitude_target_send(0,   # system time in milliseconds
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            mask,                                   # mask
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust

    def set_servo_pwm(self, id, pwm=1500):
        """ 设置伺服电机的pwm

        参数:
            id (int): 伺服电机id
            pwm (int, optional): pwm 范围是 1100-2000
        """
        mavutil.mavfile.set_servo(self.conn, id, pwm)


    def set_rc_channel_pwm(self, id, pwm=1500):
        """ 设置RC Channel的pwm,可以控制ROV的上下、前后等运动

        Args:
            id (TYPE): 通道 id
            pwm (int, optional):  pwm 范围是 1100-2000
        """
        rc_channel_values = [65535 for _ in range(8)] #8 for mavlink1
        rc_channel_values[id] = pwm
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,                # target_system
            self.conn.target_component,             # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.
    
    def set_manual_control(self,joy_list=[0]*4, buttons_list=[0]*16):
        """ 发送手动控制信号
        Set a MANUAL_CONTROL message for dealing with more control with ArduSub
        for now it is just to deal with lights under test...
        """
        x,y,z,r = 0,0,0,0#32767,32767,32767,32767
        b = 0
        for i in range(len(buttons_list)):
            b = b | (buttons_list[i]<<i)
        print("MANUAL_CONTROL_SEND : x : {}, y : {}, z : {}, r : {}, b : {}".format(x,y,z,r,b))
        #https://mavlink.io/en/messages/common.html MANUAL_CONTROL ( #69 )
        self.conn.mav.manual_control_send(
               self.conn.target_system,
                x,
                y,
                z,
                r,
                b)

    def arm_throttle(self, arm_throttle):
        """ 修改锁定状态

        Args:
            arm_throttle (bool): Arm state
        """
        if arm_throttle:
            self.conn.arducopter_arm()
        else:
            #http://mavlink.org/messages/common#MAV_CMD_COMPONENT_ARM_DISARM
            # param1 (0 to indicate disarm)
            # Reserved (all remaining params)
            self.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0, 0, 0, 0, 0, 0, 0]
            )