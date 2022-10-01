#!/usr/bin/env python3

#ros import
#import rospy
#import std_msgs.msg
#from sensor_msgs.msg import Image

import numpy as np
import cv2
import torch
from PIL import Image

# Model
model = torch.hub.load("ultralytics/yolov5","custom", path="best.pt")

cap = cv2.VideoCapture("video.mp4")

# Inference
def inference():
    while True:
        ret, frame = cap.read()
        if not ret:
            break;

        results = model(frame, size=640)  # includes NMS
        for detection in results.xyxy[0]:
            x0 = detection[0].tolist() #xmin
            y0 = detection[1].tolist() #ymin
            x1 = detection[2].tolist() #xmax
            y1 = detection[3].tolist() #ymax
            obj = detection[5].tolist() #object
            print(x0,y0,x1,y1, obj)
        #print(results.xyxy[0].tolist())  # img1 predictions (tensor)
        #print(results.pandas().xyxy[0])  # img1 predictions (pandas)
        cv2.imshow("frame",frame)
        #results.print()  

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



# Results
if __name__ == "__main__":
    inference()
