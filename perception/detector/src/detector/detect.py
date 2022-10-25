#!/usr/bin/env python3

# ros import
import rospy
from std_msgs.msg import Float32,Int16
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError

# custom msg and srv
from garbage_quick_sort_robot_msg.msg import EffectorPose, RobotState
from garbage_quick_sort_robot_msg.srv import RobotStateFbk, RobotStateFbkResponse, EffectorPoseFbk, EffectorPoseFbkRequest, EffectorPoseFbkResponse
from detection_msgs.msg import BoundingBox, BoundingBoxes, TargetCenter, TargetsInfoPerFrame, TargetsInfo
from detection_msgs.srv import FramesInfo, FramesInfoResponse

# obj detection
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from math import pi, tan
print("Using CUDA: ",torch.cuda.is_available())

# import from yolov5 submodules
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.torch_utils import select_device
from yolov5.utils.augmentations import letterbox

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
        self.model.warmup(imgsz=(1 if self.pt else bs, 3, *self.img_size))  # warmup   

        # state of robot variables
        self.active_status = False
        self.goal_status = 0
        self.end_effector_target_pose = None

        self.active_global_states = [2]
        self.camera_height = 0.568

        '''Subscribers'''
        rospy.Subscriber('/garbage_quick_sort/camera/image', Image, self.image_callback) # Get image
        rospy.Subscriber('/garbage_quick_sort/global_state', RobotState, self.global_state_callback) # Check global state: looking for 4: GetPickUpLoc

        '''Service'''
        self.detect_RobotStateFbk = rospy.Service('/garbage_quick_sort/detect_RobotStateFbk', RobotStateFbk, self.state_feedback) # 1: in progress, 2: not found, 3: found
        self.target_pose_server = rospy.Service('/garbage_quick_sort/target_pose_service', EffectorPoseFbk, self.target_pose_callback) # end_effecter_pose camera frame
        self.frames_info_server = rospy.Service('/garbage_quick_sort/frame_info', FramesInfo, self.frames_info_callback) # end_effecter_pose camera frame

        self.type_of_garbage = 0
        self.target_center = TargetCenter()
        self.targets_info = TargetsInfo()
        self.reached_target_frames_count = False
        self.target_frame_count = 0
        self.target_frame_limit = 50


        rospy.loginfo("Service started")

        self.detection_image_pub = rospy.Publisher("/garbage_quick_sort/detect_garbage", Image, queue_size=10)

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
        if msg.robot_state in self.active_global_states:
            self.active_status = True
        else:
            self.active_status = False
            self.goal_status = 0

    def target_pose_callback(self, req):
        res = EffectorPoseFbkResponse()
        res.type = self.type_of_garbage # 1: cardboard, 2: metal, 3: plastic
        res.pose_value = self.end_effector_target_pose 

        return res

    def frames_info_callback(self, req):
        res = FramesInfoResponse()
        if self.reached_target_frames_count and req:
            res.success = True
            res.frames_info = self.targets_info
            self.target_frame_count = 0
            self.reached_target_frames_count = False
            return res
        else:
            res.frames_info = self.targets_info
            res.success = False
            return res

        

    def image_callback(self, data):
        self.img_raw = data
        self.bgr_image = self.br.imgmsg_to_cv2(data,"bgr8")
        self.gray_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2GRAY)
        self.image_height, self.image_width, self.image_chanel = self.bgr_image.shape # (480,640,3)
        self.bgr_image = cv2.circle(self.bgr_image, [self.image_width//2, self.image_height//2], 2,(0,0,255),2)
        self.start_image = True

    def state_feedback(self, request):
        res = RobotStateFbkResponse()
        res.active_status = self.active_status
        res.goal_status = self.goal_status

        return res


    def frames_info_callback(self, req):
        res = FramesInfoResponse()
        if self.reached_target_frames_count and req:
            res.success = True
            res.frames_info = self.targets_info
            self.target_frame_count = 0
            self.reached_target_frames_count = False
            return res
        else:
            res.frames_info = self.targets_info
            res.success = False
            return res


    def inference(self):
        if self.start_image:

            z_value = self.camera_height

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

                    # bounding_box_area = (bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin)

                    if (bounding_box.xmax - bounding_box.xmin) > (self.image_width - 100) or (bounding_box.ymax - bounding_box.ymin) > (self.image_height - 80):
                        # print(bounding_box.xmax - bounding_box.xmin, bounding_box.ymax - bounding_box.ymin)
                        continue

                    if conf > highest_conf:
                        self.type_of_garbage = c + 1
                        self.obj_center = ((bounding_box.xmin + bounding_box.xmax) // 2,(bounding_box.ymin + bounding_box.ymax) // 2) # look for target with highest conf value
                        self.goal_status = 3 

                    bounding_boxes.bounding_boxes.append(bounding_box)

                    # Annotate the image
                    if self.view_image:  # Add bbox to image
                          # integer class
                        label = f"{self.names[c]} {conf:.2f}"
                        annotator.box_label(xyxy, label, color=colors(c, True))

                self.bgr_image = cv2.circle(self.bgr_image, [self.obj_center[0], self.obj_center[1]], 2,(0,255,0),2) # show target center point
                # Stream results
                im0 = annotator.result()
                
                if not self.is_moving():

                    x,y = self.pixel2xy(self.obj_center[0], self.obj_center[1], z_value) # calculate target x, y distance from camera center frame

                    self.end_effector_target_pose = EffectorPose()
                    self.end_effector_target_pose.x = x
                    self.end_effector_target_pose.y = y
                    self.end_effector_target_pose.z = z_value # give the same z_pose 
                    self.end_effector_target_pose.phi = -np.pi/2 # maybe software can later provide this also :P , right now irrelevant because not used

                # self.frame_info_collection(bounding_boxes, z_value)

            else:
                self.goal_status = 1 # not found/in progress

            self.detection_image_pub.publish(self.br.cv2_to_imgmsg(self.bgr_image, "bgr8"))
            cv2.imshow(str(0), self.bgr_image)
            cv2.waitKey(1)

    def pixel2xy(self, pixel_x, pixel_y, z): # take center as origin, z in meter
        z *= 100
        x = z * tan(0.08 * (pixel_x - self.image_width / 2) * pi / 180)
        y = z * tan(0.08 * (self.image_height / 2 - pixel_y) * pi / 180)
        
        transformed_x = y
        transformed_y = -x

        return transformed_x/100, transformed_y/100 #return in m

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

        if self.static_count > 5:
            return False

        else:
            return True

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


    def frame_info_collection(self, bounding_boxes, z_value):
        print(self.target_frame_count)
        self.targets_info_per_frame = TargetsInfoPerFrame()
        if self.target_frame_count == 0:
            self.targets_info.target_info.clear()

        if self.target_frame_count >= self.target_frame_limit:
            self.reached_target_frames_count = True
            # set the goal status to done
            self.goal_status = 3

        else:
            self.target_frame_count += 1
            for i, bounding_box in enumerate(bounding_boxes.bounding_boxes):
                if bounding_box.Class == "CARDBOARD":
                    object_class = 1

                elif bounding_box.Class == "METAL":
                    object_class = 2

                elif bounding_box.Class == "PLASTIC":
                    object_class = 3

                pixel_x = (bounding_box.xmin + bounding_box.xmax) // 2
                pixel_y = (bounding_box.ymin + bounding_box.ymax) // 2
                x,y = self.pixel2xy(pixel_x, pixel_y, z_value)

                self.target_center.object_class = object_class
                self.target_center.x = x
                self.target_center.y = y
                self.target_center.conf = bounding_box.probability
                self.targets_info_per_frame.target_info.append(self.target_center)

            self.targets_info_per_frame.frame_count = self.target_frame_count

            self.targets_info.target_info.append(self.targets_info_per_frame)


    # function to get the average (x,y) taking input the 100 frames from camera
    def get_detection_avg(self, frames_input):
        # first frame info is of type TargetsInfoPerFrame
        first_frame_info_l = frames_input.target_info[0].target_info
        # final array after collating 99 frames info
        final_frame_info_l = deepcopy(first_frame_info_l)
        # first frame arr and final frame arr are of type TargetCenter
        # create a list to keep track of how many times an object has been added confidence
        no_of_times_l = [1] * len(first_frame_info_l)

        # loop through the rest 99 frames and keep adding corresponding confidences to first frame array
        for i in range(1, len(frames_input.target_info)):
            # get the current frame details
            curr_frame_info_arr_l = frames_input.target_info[i].target_info
            # loop through curr_frame_info_arr_data to find the location of each object and confidence and match
            for object_num, object in enumerate(curr_frame_info_arr_l):
                object_x = object.x
                object_y = object.y
                object_conf = object.conf

                for first_object_num, first_object in enumerate(first_frame_info_l):
                    # get the x,y of this object
                    first_object_x = first_object.x
                    first_object_y = first_object.y

                    x_diff = abs(first_object_x - object_x)
                    y_diff = abs(first_object_y - object_y)

                    if ((x_diff <= self.object_x_tolerance) and (y_diff <= self.object_y_tolerance)):
                        if (first_object.object_class == object.object_class):
                            # add the confidence and (x,y) for this object
                            final_frame_info_l[first_object_num].x += object_x
                            final_frame_info_l[first_object_num].y += object_y

                            final_frame_info_l[first_object_num].conf += object_conf

                            no_of_times_l[first_object_num] += 1
                            # break out of this loop, now check for next object
                            break
                        # the object class is not same, pass
                        else:
                            pass
                    # the object is not close by, pass
                    else:
                        pass

        # now loop through the entire final_frame_info_l to find the highest confidence object
        max_conf_obj = final_frame_info_l[0]
        max_conf_obj_num = 0
        for j_num, j in enumerate(final_frame_info_l):
            if (j.conf > max_conf_obj.conf):
                max_conf_obj = final_frame_info_l[j_num]
                max_conf_obj_num = j_num
            else:
                pass

        # now get the average (x,y) for the max_conf_obj
        avg_x = max_conf_obj.x / no_of_times_l[max_conf_obj_num]
        avg_y = max_conf_obj.y / no_of_times_l[max_conf_obj_num]

        return avg_x, avg_y