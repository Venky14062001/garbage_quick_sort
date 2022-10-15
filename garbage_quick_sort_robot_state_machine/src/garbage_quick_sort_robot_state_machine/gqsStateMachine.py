#!/usr/bin/env python3

'''This class acts as the state machine for the GQS Robot, instructing other nodes on what state robot is in and getting feedback from concerned nodes'''

import rospy
import enum
from garbage_quick_sort_robot_msg.msg import RobotState, EffectorPose
from garbage_quick_sort_robot_msg.srv import RobotStateFbk, RobotStateFbkRequest, RobotStateFbkResponse, EffectorPoseFbk, EffectorPoseFbkRequest, EffectorPoseFbkResponse

import numpy as np
class RobotStateEnum(enum.Enum):
    StayIdle = 0
    GoHome = 1
    TransHomePickHome = 2
    GoPickHome = 3
    GetPickUpLoc = 4
    GoPickUpLoc = 5
class GarbageQuickSortRobotStateMachine:
    def __init__(self):
        rospy.init_node("GarbageQuickSortStateMachine", anonymous=True)
        self.state = RobotStateEnum.GoHome

        # define required poses here
        self.home_pose = EffectorPose()
        self.home_pose.x = 0.53099
        self.home_pose.y = 0.000
        self.home_pose.z = 0.131
        self.home_pose.phi = 0.0

        self.pick_home_pose = EffectorPose()
        self.pick_home_pose.x = 0.205
        self.pick_home_pose.y = 0.02
        self.pick_home_pose.z = 0.092
        self.pick_home_pose.phi = -1.571

        # this stores the transformed camera target pose to be commanded to GQS Robot
        self.global_target_pose = EffectorPose()
        self.global_target_pose.x = None
        self.global_target_pose.y = None
        self.global_target_pose.z = None
        self.global_target_pose.phi = None

        self.state_publisher =rospy.Publisher(
            "/garbage_quick_sort/global_state", 
            RobotState, queue_size=10
        )

        # pose publisher
        self.pose_publisher = rospy.Publisher(
            "/garbage_quick_sort/end_effector_pose",
            EffectorPose, queue_size=1
        )

        # create the required client handlers
        rospy.wait_for_service("/garbage_quick_sort/go_robot_service")
        try:
            self.go_robot_client_obj = rospy.ServiceProxy("/garbage_quick_sort/go_robot_service", RobotStateFbk)
        except rospy.ServiceException as e:
            print("Go Robot service object failed: ", e)

        # create client handlers for detection node feedback
        rospy.wait_for_service("/garbage_quick_sort/detect_RobotStateFbk")
        try:
            self.detect_client_obj = rospy.ServiceProxy("/garbage_quick_sort/detect_RobotStateFbk", RobotStateFbk)
        except rospy.ServiceException as e:
            print("YOLO Robot service object failed: ", e)

        # create client handler to get target_pose in camera frame
        rospy.wait_for_service("/garbage_quick_sort/target_pose_service")
        try:
            self.target_pose_client_obj = rospy.ServiceProxy("/garbage_quick_sort/target_pose_service", EffectorPoseFbk)
        except rospy.ServiceException as e:
            print("YOLO target pose service object failed: ", e)

        # create client handler to get global pose of robot
        rospy.wait_for_service("/garbage_quick_sort/effector_pose_service")
        try:
            self.global_pose_client_obj = rospy.ServiceProxy("/garbage_quick_sort/effector_pose_service", EffectorPoseFbk)
        except rospy.ServiceException as e:
            print("Go robot global pose service object failed: ", e)

        print("Garbage Quick Sort state machine activated :) !")

    def run(self):
        # add a statement to get input on whether to start program
        user_input = input("Ready to get moving..? ('yes' or 'no')")

        if (user_input == "yes"):
            while not rospy.is_shutdown():
                # conditional statements to decide and execute certain services
                if (self.state == RobotStateEnum.StayIdle):
                    pass

                elif (self.state == RobotStateEnum.GoHome):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoHome failed: ", e)
                        continue
                    
                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoHome state! Changing to TransHomePickHome state now")
                        self.state = RobotStateEnum.TransHomePickHome
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoHome state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.home_pose)
                        pass
                    else:
                        print("GoHome state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.TransHomePickHome):
                    self.state = RobotStateEnum.GoPickHome
                    rospy.sleep(0.5)

                elif (self.state == RobotStateEnum.GoPickHome):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoPickHome failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoPickHome state! Moving to next state GetPickUpLoc")
                        self.state = RobotStateEnum.GetPickUpLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.pick_home_pose)
                        pass
                    else:
                        print("GoPickHome state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GetPickUpLoc):
                    req = RobotStateFbkRequest()
                    try:
                        detection_fbk = self.detect_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GetPickUpLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((detection_fbk.active_status == True) and (detection_fbk.goal_status == 3)):
                        # inference node has found a target spot! YAY! Get the target pose
                        req = EffectorPoseFbkRequest()
                        try:
                            target_fbk = self.target_pose_client_obj(req)
                        except rospy.ServiceException as e:
                            print("Service call target pose failed: ", e)
                            continue

                        # get the current global pose to form transform matrix
                        global_pose_req = EffectorPoseFbkRequest()
                        try:
                            global_pose_fbk = self.global_pose_client_obj(global_pose_req)
                        except rospy.ServiceException as e:
                            print("Service call global pose failed: ", e)
                            continue

                        # now use the (x, y) from target_fbk, transform to global state and store it
                        transform_matrix = np.array([[np.cos(global_pose_fbk.pose_value.phi), -np.sin(global_pose_fbk.pose_value.phi), 0, global_pose_fbk.pose_value.x], 
                                                        [np.sin(global_pose_fbk.pose_value.phi), np.cos(global_pose_fbk.pose_value.phi), 0, global_pose_fbk.pose_value.y],
                                                        [0, 0, 1, global_pose_fbk.pose_value.z],
                                                        [0, 0, 0, 1]])

                        target_pose_array = np.array([target_fbk.pose_value.x, target_fbk.pose_value.y, target_fbk.pose_value.z, 1])

                        global_target = np.matmul(transform_matrix, target_pose_array)
                        self.global_target_pose.x = global_target[0]
                        self.global_target_pose.y = global_target[1]
                        self.global_target_pose.z = global_target[2]
                        self.global_target_pose.phi = -1.571

                        self.state = RobotStateEnum.GoPickUpLoc

                    elif ((detection_fbk.active_status == True) and (detection_fbk.goal_status == 2)):
                        print("GetPickUpLoc state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((detection_fbk.active_status == True) and (detection_fbk.goal_status == 1)):
                        pass
                    elif ((detection_fbk.active_status == True) and (detection_fbk.goal_status == 0)):
                        # recall the server to get target pose (CHECK HOW TO IMPLEMENT)
                        pass
                    else:
                        print("GetPickUpLoc state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoPickUpLoc):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoPickUpLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        # reset global_target_pose to None
                        self.global_target_pose = None
                        print("Completed GoPickUpLoc state! Moving to next state StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickUpLoc state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.global_target_pose)
                        pass
                    else:
                        print("GoPickUpLoc state node not activated!")
                        pass

                # publish state 
                current_state = RobotState()
                current_state.robot_state = self.state.value
                self.state_publisher.publish(current_state)

                rospy.sleep(0.5)
            
        elif (user_input == "no"):
            return

        else:
            print("Please enter valid input :) !")
            return

        