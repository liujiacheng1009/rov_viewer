#!/bin/usr/env python3

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import cv2
import os 
import numpy as np 

class Calib:
    def __init__(self):
        self.K = None
        self.D = None
        self.image_size = (640,480)
        self.marker_type = 0 ## 0为chessboard, 1为arucoboard
        self.camera_type = 0 ## 0为普通单目，1为鱼眼
        self.corners_w = []
        self.corners_i = []
        self.pattern_size = [6,9]
        self.square_size = 0.05
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50) ## marker 字典
        
        self.buildBoard()

    def buildBoard(self):
        ## 创建arucoboard
        ## 计算board的世界坐标，注意board比pattern小一圈
        self.corners_w = []
        self.board = cv2.aruco.CharucoBoard_create(self.pattern_size[0]+1, self.pattern_size[1]+1, 1, 0.8, self.dict)
        for i in range(self.pattern_size[0]):
            for j in range(self.pattern_size[1]):
                self.corners_w.append([[i*self.square_size,j*self.square_size, 0]])
        self.corners_w = np.array(self.corners_w,np.float32)

    def calibrate(self,images):
        n = len(images)
        self.image_size = images[0].shape[:2]
        for i in range(n):
            image = images[i]
            if self.marker_type == 0:
                ret, corners = cv2.findChessboardCorners(image, (self.pattern_size[0],self.pattern_size[1]))
                if ret and len(corners) == self.pattern_size[0]*self.pattern_size[1]:
                    self.corners_i.append(np.array(corners))
            elif self.marker_type == 1:
                corners, ids, _ = cv2.aruco.detectMarkers(image, self.dict)
                if len(corners) == self.pattern_size[0]*self.pattern_size[1]:
                    _,corners,_ = cv2.aruco.interpolateCornersCharuco(corners, ids, image,self.board)
                    self.corners_i.append(np.array(corners))
        if self.camera_type == 0:
            ret, self.K, self.D,_,_= cv2.calibrateCamera([self.corners_w]*n,self.corners_i,self.image_size,None,None)
        elif self.camera_type == 1:
            ret, self.K, self.D,_,_ = cv2.fisheye.calibrate([self.corners_w]*n,self.corners_i ,self.image_size, None,None)

        return ret 



if __name__ == "__main__":

    calib = Calib()
    calib.square_size = 0.031
    calib.buildBoard()
    images = []
    path = "images/"
    for file in os.listdir(path):
        image = cv2.imread(path+file)
        images.append(image)
    calib.calibrate(images)
    print(calib.K)
    print(calib.D)
