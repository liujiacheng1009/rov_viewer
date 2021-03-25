#!/usr/bin/env python3
import cv2 
import numpy as np 

path = "/home/bluerov/Downloads/catkin_ws/src/rov_viewer/src/gui/image_page/yolo/model/"
modelConfiguration = path+ 'yolov3-tiny.cfg'
modelWeights = path+ 'yolov3-tiny.weights'
classesFile = path + 'coco.names'

class YOLO:
    def __init__(self):
        self.confThreshold = 0.5
        self.nms_threshold = 0.3 
        self.whT = 320
        self.classNames = []
        with open(classesFile, 'rt') as f:
            self.classNames = f.read().rstrip('\n').split('\n')
        self.net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect(self,image):
        blob = cv2.dnn.blobFromImage(image,1/255,(self.whT,self.whT),[0,0,0],1,crop=False)
        self.net.setInput(blob)
        layerNames = self.net.getLayerNames()
        outputNames = [layerNames[i[0]-1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(outputNames)
        hT,wT,cT = image.shape
        bbox = []

        #class Ids and their confidence value
        classIds = []
        confs = []
        for output in outputs:
            for det in output:
                #remove first 5 elements find the value of the height value
                scores = det[5:]
                #index
                classId = np.argmax(scores)
                #value of that index
                confidence = scores[classId]

                #filtering object
                if confidence > self.confThreshold:
                    #save width,height,x,y(are in decimals so we have to multiple them by our actual image size)
                    w,h=int(det[2]*wT),int(det[3]*hT)
                    #the center point(divide by 2 and substract)
                    x,y=int(det[0]*wT - w/2) , int(det[1]*hT - h/2)
                    bbox.append([x,y,w,h])
                    classIds.append(classId)
                    confs.append(float(confidence))

        indices = cv2.dnn.NMSBoxes(bbox,confs,self.confThreshold,self.nms_threshold)
        for i in indices:
            #to remove the extra bracket
            i = i[0]
            box = bbox[i]
            #extract x,y,width,height
            x,y,w,h = box[0],box[1],box[2],box[3]
            #drawing the box
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,255),2)
            #print name and confidence level
            cv2.putText(image,f'{self.classNames[classIds[i]].upper()} {int(confs[i]*100)}%',
                        (x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
        