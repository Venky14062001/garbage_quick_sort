#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32, Int16MultiArray, String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError
import time

# obj detection
import cv2
import torch
import numpy as np
from math import pi, tan
print("Using CUDA: ",torch.cuda.is_available())

# Inference
class Detection:
    def __init__(self):
        self.weights_path = rospy.get_param('~weights_path')

        self.model = torch.hub.load("ultralytics/yolov5","custom", path=self.weights_path)
        #self.model.classes=[0] #['BIODEGRADABLE', 'CARDBOARD', 'GLASS', 'METAL', 'PAPER', 'PLASTIC']

        self.br = CvBridge()
        self.distance = 10

        self.start_image = False

        rospy.Subscriber('/zedm/zed_node/left/image_rect_color', Image, self.image_callback)
        rospy.Subscriber('/distance', Float32, self.distance_callback)


        self.pub_target_angle = rospy.Publisher('/cmd_out/target_angle', Float32, queue_size=10)
        self.start_service = rospy.Service('detection_service', SetBool, self.trigger_publish)
        self.pub_the_msg = True 

        self.boundaries = { # hsv color boundaries
            'red' : np.array([[0,120,5], [5,255,255], [161, 125, 5], [179, 255, 255]]),  # plastic
            'blue' : np.array([[98, 109, 2], [116, 255, 255]]),   # paper
            'orange' : np.array([[20,90,20],[30,255,255], [6,100,150],[14,255,255]])  # cans

        }

        self.bgr_colors = {'red':(0,0,255), 'blue':(255,0,0), 'orange':(0,140,255)}


        self.x0 = 0
        self.y0 = 0
        self.x1 = 0
        self.y1 = 0
        self.conf = 0.0
        self.obj = 10
        self.obj_center = [0,0]
        self.box_xywh = [0,0,0,0]
        self.box_center = [0,0]


    def image_callback(self, data):
        self.bgr_image = self.br.imgmsg_to_cv2(data,"bgr8")
        self.image_height, self.image_width, self.image_chanel = self.bgr_image.shape # (480,640,3)
        self.bgr_image = cv2.circle(self.bgr_image, [self.image_width//2, self.image_height//2], 2,(0,0,255),2)
        self.start_image = True


    def distance_callback(self, msg):
        self.distance = msg.data


    def inference(self):
        results = self.model(self.bgr_image, size=320)  # includes NMS
        outputs = results.xyxy[0].cpu()
        if len(outputs) > 100:
            for i,detection in enumerate(outputs):
                self.x0 = int(outputs[i][0]) #xmin
                self.y0 = int(outputs[i][1]) #ymin
                self.x1 = int(outputs[i][2]) #xmax
                self.y1 = int(outputs[i][3]) #ymax
                self.conf = float(outputs[i][4])
                self.obj = int(outputs[i][5]) #object number
                self.obj_center = ((self.x0+self.x1)//2,(self.y0+self.y1)//2)
                #self.bgr_image = cv2.circle(self.bgr_image, [self.obj_center[0], self.obj_center[1]], 2,(0,0,255),2)

                if self.pub_the_msg == True:
                    pass  

                if self.conf > 0.3:
                    self.bgr_image = cv2.circle(self.bgr_image, [self.obj_center[0], self.obj_center[1]], 2,(0,0,255),2)
                    
                    cv2.rectangle(self.bgr_image,(self.x0, self.y0),(self.x1,self.y1),(0,255,0),3)
                
                break

        #distance = 15 #cm

        #length = distance * (tan((self.image_width / 2 - self.x0)* 0.08 * pi / 180) + tan((self.x1 - self.image_width / 2) * 0.08* pi /180))
        self.box_contour()
        self.color_detect()
        print(self.ratio_calculate())

        cv2.imshow("frame",self.bgr_image)
        if cv2.waitKey(1) == ord('q'):  # q to quit
            cv2.destroyAllWindows()
            raise StopIteration  



    def color_detect(self):
        blur = cv2.GaussianBlur(self.bgr_image, (11, 11), 0) # reduce noises, smoothing image
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) # convert BGR image to HSV image
        center = [0,0]
        for color, code in self.boundaries.items():
            if color == 'red' or color == 'orange':
                low1, high1, low2, high2 = code
                mask1 = cv2.inRange(hsv, low1, high1)
                mask2 = cv2.inRange(hsv, low2, high2)
                mask = mask1 + mask2
            else:
                low, high = code
                mask = cv2.inRange(hsv, low, high)

            kernel = np.ones((10,10),np.uint8) # Unsigned(no negative value) int 8bits(0-255)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # remove false positives. remove pixels(noises) from image (outside detected shape)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # remove false negatives. inside detected shape

            cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)


            for index, con in enumerate(cnts):
                (x,y),radius = cv2.minEnclosingCircle(con)
                if radius < 2: # ignore noises 
                    pass
                else:
                    center = [int(x),int(y)]
                    cv2.drawContours(self.bgr_image, cnts, -1, self.bgr_colors[color], 5)
                    cv2.putText(self.bgr_image,color, center, cv2.FONT_HERSHEY_SIMPLEX, 0.6,self.bgr_colors[color],2)
            break

        self.obj_center = center

        try:
            #print(self.pixel2xy(x,y,self.distance))
            self.pixel2xy(x,y,self.distance)
        except UnboundLocalError:
            pass
            
            

    def pixel2xy(self, pixel_x, pixel_y, distance): # take center as origin, all units in cm
        x = distance * tan(0.08 * (pixel_x - self.image_width / 2) * pi / 180)
        y = distance * tan(0.08 * (self.image_height / 2 - pixel_y) * pi / 180)

        return x,y



    def trigger_publish(self, request):
        if request.data:
            self.pub_the_msg = True
            return SetBoolResponse(True, "Publishing data...")
        else:
            self.pub_the_msg = False
            return SetBoolResponse(False, "Keep Quiet...")


    def box_contour(self):
        self.box_xywh = [0,0,0,0]
        grayImage=cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2GRAY)
        estimatedThreshold, thresholdImage=cv2.threshold(grayImage,125,255,cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresholdImage,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)

        for i, c in enumerate(contours):
            x,y,w,h = cv2.boundingRect(c)

            if w >= self.image_width - 50 or h >= self.image_height - 50:
                continue

            if w*h > self.box_xywh[2] * self.box_xywh[3]:
                self.box_xywh = [x,y,w,h]

        cv2.rectangle(self.bgr_image,(self.box_xywh[0],self.box_xywh[1]),(self.box_xywh[0] + self.box_xywh[2],self.box_xywh[1] + self.box_xywh[3]),(200,0,0),2)
        self.box_center = [(self.box_xywh[0] + self.box_xywh[2]) // 2, (self.box_xywh[1] + self.box_xywh[3]) // 2]
        


    def ratio_calculate(self):

        try:
            x_ratio = (self.obj_center[0] - self.box_xywh[0]) / self.box_xywh[2]
            y_ratio = (self.obj_center[1] - self.box_xywh[1]) / self.box_xywh[3]

            if self.obj_center[0] < self.box_center[0]:
                x_ratio *= -1

            if self.obj_center[1] > self.box_center[1]:
                y_ratio *= -1

            return x_ratio, y_ratio


        except ZeroDivisionError:
            return 0,0



# Results
if __name__ == "__main__":
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(15)
    detection = Detection()
    try:
        
        while not rospy.is_shutdown():
            if detection.start_image:
                
                detection.inference()
                #detection.pub_the_msg = True
            rate.sleep()

    except (KeyboardInterrupt, StopIteration):
        pass
