#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32,Int16
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError

# custom msg and srv
from garbage_quick_sort_robot_msg.msg import EffectorPose, RobotState
from garbage_quick_sort_robot_msg.srv import RobotStateFbk, RobotStateFbkResponse
from detection_msgs.msg import BoundingBox, BoundingBoxes
from detection_msgs.srv import yolo, yoloResponse

# obj detection
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from math import pi, tan
print("Using CUDA: ",torch.cuda.is_available())

from pathlib import Path
import os
import sys

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


class Detection:
    def __init__(self):
        '''YOLOv5'''
        self.br = CvBridge()
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        weights = rospy.get_param("~weights")
        self.start_image = False
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup(imgsz=(1 if self.pt else bs, 3, *self.img_size), half=self.half)  # warmup   


        '''Subscribers'''
        rospy.Subscriber('/garbage_quick_sort/camera/image', Image, self.image_callback) # Get image
        rospy.Subscriber('/garbage_quick_sort/global_state', RobotState, self.global_state_callback) # Check global state: looking for 2: GetPickUpLoc
        self.global_state = 2 ################################## for testing


        '''Service'''
        # self.detection_service = rospy.Service('detection_service', yolo, self.return_target_xy) # request: z (absolute), response: x, y (camera frame) 
        self.response_RobotStateFbk = rospy.Service('/garbage_quick_sort/response_RobotStateFbk', RobotStateFbk, self.state_feedback) # 1: in progress, 2: not found, 3: found
        rospy.loginfo("Service started")


        '''Client'''
        rospy.wait_for_service('/garbage_quick_sort/z_service')
        z_client = rospy.ServiceProxy('/garbage_quick_sort/z_service', zFbk) 


        '''Publisher'''
        self.pub_target_pose = rospy.Publisher('/garbage_quick_sort/camera_frame/end_effector_pose', EffectorPose, queue_size=10) # end_effecter_pose camera frame
        self.end_effector_pose_msg = EffectorPose()


        '''Color Detection'''
        self.boundaries = { # hsv color boundaries
            'red' : np.array([[0,120,5], [5,255,255], [161, 125, 5], [179, 255, 255]]),  # plastic
            'blue' : np.array([[98, 109, 2], [116, 255, 255]]), # paper
            'orange' : np.array([[20,90,20],[30,255,255], [6,100,150],[14,255,255]])  # cans

        }
        self.bgr_colors = {'red':(0,0,255), 'blue':(255,0,0), 'orange':(0,140,255)}


        '''Box Contour Detection'''
        self.obj_center = [0,0]
        self.box_xywh = [0,0,0,0]
        self.box_center = [0,0]
        self.found_garbage = False


        '''Motion Detection'''
        self.static_back = None
        self.prev_gray = None
        self.static_count = 0


    def global_state_callback(self, msg):
        self.global_state = msg.robot_state


    def image_callback(self, data):
        self.img_raw = data
        self.bgr_image = self.br.imgmsg_to_cv2(data,"bgr8")
        self.gray_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2GRAY)
        self.image_height, self.image_width, self.image_chanel = self.bgr_image.shape # (480,640,3)
        self.bgr_image = cv2.circle(self.bgr_image, [self.image_width//2, self.image_height//2], 2,(0,0,255),2)
        self.start_image = True


    def state_feedback(self, request):
        if request:
            return RobotStateFbkResponse(self.active_status, self.goal_status)



    def inference(self):
        if self.start_image and self.global_state == 2: # GetPickUpLoc = 2
            self.goal_status = 1 # in progress
            self.active_status = True

            if self.is_moving():
                return
                
            z = z_client()
            # z = 0.2  # for testing

            im, im0 = self.preprocess(self.bgr_image)
            im = torch.from_numpy(im).to(self.device) 
            im = im.half() if self.half else im.float()
            im /= 255
            if len(im.shape) == 3:
                im = im[None]

            results = self.model(im, augment=False, visualize=False)
            results = non_max_suppression(
                results, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
            )
            det = results[0].cpu().numpy()
            bounding_boxes = BoundingBoxes()
            bounding_boxes.header = self.img_raw.header
            bounding_boxes.image_header = self.img_raw.header
            annotator = Annotator(self.bgr_image, line_width=self.line_thickness, example=str(self.names))

            highest_conf = 0.0

            if len(det): # found garbage
                self.goal_status = 3 # found
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    bounding_box = BoundingBox()
                    c = int(cls)
                    # Fill in bounding box message
                    bounding_box.Class = self.names[c]
                    bounding_box.probability = conf 
                    bounding_box.xmin = int(xyxy[0])
                    bounding_box.ymin = int(xyxy[1])
                    bounding_box.xmax = int(xyxy[2])
                    bounding_box.ymax = int(xyxy[3])

                    bounding_box_area = (bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin)
                    if (bounding_box.xmax - bounding_box.xmin) > 600 or (bounding_box.ymax - bounding_box.ymin) > 400:
                        continue

                    if conf > highest_conf:
                        self.obj_center = ((bounding_box.xmin + bounding_box.xmax) // 2,(bounding_box.ymin + bounding_box.ymax) // 2) # look for target with highest conf value

                    bounding_boxes.bounding_boxes.append(bounding_box)

                    # Annotate the image
                    if self.view_image:  # Add bbox to image
                          # integer class
                        label = f"{self.names[c]} {conf:.2f}"
                        annotator.box_label(xyxy, label, color=colors(c, True))

                self.bgr_image = cv2.circle(self.bgr_image, [self.obj_center[0], self.obj_center[1]], 2,(0,0,255),2) # show target center point
                # Stream results
                im0 = annotator.result()

                x,y = self.pixel2xy(self.obj_center[0], self.obj_center[1], z) # calculate target x, y distance from camera center frame
                self.publish_target_pose(x,y,z) # publish end effector pose

            else:
                self.goal_status = 2 # not found

            cv2.imshow(str(0), self.bgr_image)
            cv2.waitKey(1)
        else:
            self.goal_status = 0 # not detection state
            self.active_status = False




    def pixel2xy(self, pixel_x, pixel_y, z): # take center as origin, z in meter
        z *= 100
        x = z * tan(0.08 * (pixel_x - self.image_width / 2) * pi / 180)
        y = z * tan(0.08 * (self.image_height / 2 - pixel_y) * pi / 180)

        return x/100, y/100 #return in m


    def publish_target_pose(self,x,y,z,phi=-90*pi/180):
        self.end_effector_pose_msg.x = x # meter
        self.end_effector_pose_msg.y = y # meter
        self.end_effector_pose_msg.z = z # meter
        self.end_effector_pose_msg.phi = phi # rad
        
        self.pub_target_pose.publish(self.end_effector_pose_msg)


    def preprocess(self, img):
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0


    def is_moving(self):
        self.gray_image = cv2.GaussianBlur(self.gray_image, (21, 21), 0)

        if self.static_back is None:
            self.static_back = self.gray_image
            self.prev_gray = self.gray_image
            return
        else:
            self.static_back = self.prev_gray
        self.prev_gray = self.gray_image
    
        diff_frame = cv2.absdiff(self.static_back, self.gray_image)
        thresh_frame = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)[1]
        thresh_frame = cv2.dilate(thresh_frame, None, iterations = 2)

        # print(np.sum(thresh_frame, dtype=np.int32))
        if np.sum(thresh_frame, dtype=np.int32) > 2000: # moving
            self.static_count = 0

        else:# static
            self.static_count += 1

        if self.static_count > 10:
            return False

        else:
            return True


######################################################################## Not using ###################################################################################
'''
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

        # try:
        #     #print(self.pixel2xy(x,y,self.distance))
        #     self.pixel2xy(x,y,self.distance)
        # except UnboundLocalError:
        #     pass


    def return_target_xy(self, request): #get z(float),  return x,y float meter status(T/F) 
        if request.z:
            if self.found_garbage:
                target_x, target_y = self.pixel2xy(self.obj_center[0],self.obj_center[1], request.z)
                return yoloResponse(target_x, target_y ,True)

            else:
                return yoloResponse(0, 0 ,False)
'''


######################################################################## Not using ###################################################################################



# Results
if __name__ == "__main__":
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(15)
    detection = Detection()
    try:
        while not rospy.is_shutdown():
            detection.inference() 
            rate.sleep()
    except (KeyboardInterrupt, StopIteration):
        pass
