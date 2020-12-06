#!/usr/bin/env python3   
from pymavlink import mavutil


device='udp:192.168.2.1:14550'
baudrate=115200
conn = mavutil.mavlink_connection(device, baud=baudrate)

def get_data(self):
        """ Return data

        Returns:
            TYPE: Dict
        """
        return self.data

def get_all_msgs():
        """ Return all mavlink messages

        Returns:
            TYPE: dict
        """
        msgs = []
        while True:
            msg = conn.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

DATA = get_all_msgs()
print(DATA)
