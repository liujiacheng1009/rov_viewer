#!/usr/bin/env python3
import sys 
import os 
curr_dir = os.getcwd()
sys.path.append(curr_dir +"/src/bluerov")

from gst_reader import GSTReader
import cv2 

if __name__ == '__main__':
    video = GSTReader()
    cv2.namedWindow("frame",0)
    cv2.resizeWindow("frame", 640, 480)
    while True:
        if not video.frame_available():
            continue

        frame = video.frame()
        
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
