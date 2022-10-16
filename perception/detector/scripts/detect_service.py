#!/usr/bin/env python3

#ros import
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detector.msg import *
from detector.srv import *

import cv2
import torch
#from PIL import Image

# Model
model = torch.hub.load("ultralytics/yolov5","custom", path="best.pt")

#cap = cv2.VideoCapture("video.mp4")

# Inference
def callback(request):
    br = CvBridge()
   
    # Convert ROS Image message to OpenCV image
    frame = br.imgmsg_to_cv2(request.image)

    results = model(frame, size=640)  # includes NMS
    for detection in results.xyxy[0]:
        x0 = detection[0].tolist() #xmin
        y0 = detection[1].tolist() #ymin
        x1 = detection[2].tolist() #xmax
        y1 = detection[3].tolist() #ymax
        obj = detection[5].tolist() #object number
        return yoloResponse([x0,y0,x1,y1,obj])
    #print(results.xyxy[0].tolist())  # img1 predictions (tensor)
    #print(results.pandas().xyxy[0])  # img1 predictions (pandas)
    #cv2.imshow("frame",frame)
    #results.print()  


def inference():
    rospy.init_node('image', anonymous=True)
    service = rospy.Service('inference', yolo, callback)
    rospy.spin()

# Results
if __name__ == "__main__":
    inference()
