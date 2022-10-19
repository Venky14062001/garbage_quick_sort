#!/usr/bin/env python3

'''This class acts as the state machine for the GQS Robot, instructing other nodes on what state robot is in and getting feedback from concerned nodes'''

import rospy
import enum
from garbage_quick_sort_robot_msg.msg import RobotState, EffectorPose
from garbage_quick_sort_robot_msg.srv import RobotStateFbk, RobotStateFbkRequest, RobotStateFbkResponse, EffectorPoseFbk, EffectorPoseFbkRequest, EffectorPoseFbkResponse

import numpy as np
class RobotStateEnum(enum.Enum):
    StayIdle = 0
    GoPickHome = 1
    GetPickUpLoc = 2
    GoPickUpLoc = 3
    TransGoPickUpLocStartMoveDown = 4
    StartMoveDown = 5 # this state force sensor activated, suction activated
    TransStartMoveDownGoPickUpLocAgain = 6
    GoPickUpLocAgain = 7
    TransGoPickUpLocAgainGoDropLoc = 8
    GoDropLoc1 = 9
    GoDropLoc2 = 10
    GoDropLoc3 = 11
class GarbageQuickSortRobotStateMachine:
    def __init__(self):
        rospy.init_node("GarbageQuickSortStateMachine", anonymous=True)
        self.state = RobotStateEnum.GoPickHome #RobotStateEnum.GoPickHome

        # define required poses here
        self.pick_home_pose = EffectorPose()
        self.pick_home_pose.x = 0.125
        self.pick_home_pose.y = 0.03
        self.pick_home_pose.z = 0.15
        self.pick_home_pose.phi = -1.571

        # 1: carboard, 2: metal, 3: plastic

        self.drop1_home_pose = EffectorPose()
        self.drop1_home_pose.x = 0.175
        self.drop1_home_pose.y = 0.20
        self.drop1_home_pose.z = 0.15
        self.drop1_home_pose.phi = -1.571 

        self.drop2_home_pose = EffectorPose()
        self.drop2_home_pose.x = -0.025
        self.drop2_home_pose.y = 0.20
        self.drop2_home_pose.z = 0.15
        self.drop2_home_pose.phi = -1.571 

        self.drop3_home_pose = EffectorPose()
        self.drop3_home_pose.x = 0.075
        self.drop3_home_pose.y = 0.20
        self.drop3_home_pose.z = 0.15
        self.drop3_home_pose.phi = -1.571 

        # this stores the target type
        self.target_type = None

        # this stores the transformed camera target pose to be commanded to GQS Robot
        self.global_target_pose = EffectorPose()
        self.global_target_pose.x = None
        self.global_target_pose.y = None
        self.global_target_pose.z = None
        self.global_target_pose.phi = None

        # this stores the location to keep moving down to pick garbage GQS Robot
        self.down_global_target_pose = EffectorPose()
        self.down_global_target_pose.x = None
        self.down_global_target_pose.y = None
        self.down_global_target_pose.z = None
        self.down_global_target_pose.phi = None

        # store the camera center location
        self.camera_home_pose = EffectorPose()
        self.camera_home_pose.x = 0.405
        self.camera_home_pose.y = 0.070
        self.camera_home_pose.z = 0.15
        self.camera_home_pose.phi = -1.571 

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

        print("Garbage Quick Sort state machine activated :) !")

    def run(self):
        # add a statement to get input on whether to start program
        user_input = input("Ready to get moving..? ('yes' or 'no')")

        if (user_input == "yes"):
            while not rospy.is_shutdown():
                # conditional statements to decide and execute certain services
                if (self.state == RobotStateEnum.StayIdle):
                    pass

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

                        self.target_type = target_fbk.type
                        target_pose_array = np.array([target_fbk.pose_value.x, target_fbk.pose_value.y, 0, 1])

                        global_target = np.array([target_pose_array[0] + self.camera_home_pose.x, target_pose_array[1] + self.camera_home_pose.y, 0.15, -1.571])

                        self.global_target_pose.x = global_target[0]
                        self.global_target_pose.y = global_target[1]
                        self.global_target_pose.z = global_target[2]
                        self.global_target_pose.phi = -1.571

                        self.down_global_target_pose.x = global_target[0]
                        self.down_global_target_pose.y = global_target[1]
                        self.down_global_target_pose.z = 0.01 # putting a very low target, need to test
                        self.down_global_target_pose.phi = -1.571 

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
                        print("Completed GoPickUpLoc state! Moving to next state StartMoveDown")
                        self.state = RobotStateEnum.TransGoPickUpLocStartMoveDown
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

                elif (self.state == RobotStateEnum.TransGoPickUpLocStartMoveDown):
                    self.state = RobotStateEnum.StartMoveDown
                    print("Changing state to StartMoveDown")

                # the ToolBox node returns success if the suction is activated, then state changes
                # other approach: use state machine to subscribe directly to suction status with the ToolBox node returning success from the start itself
                elif (self.state == RobotStateEnum.StartMoveDown):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call StartMoveDown failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed StartMoveDown state! Moving to next state TransStartMoveDownGoPickUpLocAgain")
                        self.state = RobotStateEnum.TransStartMoveDownGoPickUpLocAgain
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("StartMoveDown state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.down_global_target_pose)
                        pass
                    else:
                        print("StartMoveDown state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.TransStartMoveDownGoPickUpLocAgain):
                    self.state = RobotStateEnum.GoPickUpLocAgain
                    print("Changing state to GoPickUpLocAgain")

                elif (self.state == RobotStateEnum.GoPickUpLocAgain):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoPickUpLocAgain failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        # reset global_target_pose and down_global_target_pose to None
                        self.global_target_pose.x = None
                        self.global_target_pose.y = None
                        self.global_target_pose.z = None
                        self.global_target_pose.phi = None

                        self.down_global_target_pose.x = None
                        self.down_global_target_pose.y = None
                        self.down_global_target_pose.z = None
                        self.down_global_target_pose.phi = None

                        print("Completed GoPickUpLocAgain state! Moving to next state TransGoPickUpLocAgainGoDropLoc")
                        self.state = RobotStateEnum.TransGoPickUpLocAgainGoDropLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickUpLocAgain state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.global_target_pose)
                        pass
                    else:
                        print("GoPickUpLocAgain state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.TransGoPickUpLocAgainGoDropLoc):
                    if self.target_type == 1: 
                        self.state = RobotStateEnum.GoDropLoc1
                        print("Changing state to GoDropLoc1")
                        # reset target_type
                        self.target_type = None
                    elif self.target_type == 2:
                        self.state = RobotStateEnum.GoDropLoc2
                        print("Changing state to GoDropLoc2")
                        # reset target_type
                        self.target_type = None
                    elif self.target_type == 3:
                        self.state = RobotStateEnum.GoDropLoc3
                        print("Changing state to GoDropLoc3")
                        # reset target_type
                        self.target_type = None
                    else:
                        print("Incorrect garbage type received from detect node")

                elif (self.state == RobotStateEnum.GoDropLoc1):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        # reset global_target_pose to None
                        self.global_target_pose.x = None
                        self.global_target_pose.y = None
                        self.global_target_pose.z = None
                        self.global_target_pose.phi = None
                        print("Completed GoDropLoc1 state! Moving to next state GoPickHome")
                        self.state = RobotStateEnum.GoPickHome
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoDropLoc state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.drop1_home_pose)
                        pass
                    else:
                        print("GoDropLoc1 state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoDropLoc2):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        # reset global_target_pose to None
                        self.global_target_pose.x = None
                        self.global_target_pose.y = None
                        self.global_target_pose.z = None
                        self.global_target_pose.phi = None
                        print("Completed GoDropLoc2 state! Moving to next state GoPickHome")
                        self.state = RobotStateEnum.GoPickHome
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoDropLoc state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.drop2_home_pose)
                        pass
                    else:
                        print("GoDropLoc2 state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoDropLoc3):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and StayIdle
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        # reset global_target_pose to None
                        self.global_target_pose.x = None
                        self.global_target_pose.y = None
                        self.global_target_pose.z = None
                        self.global_target_pose.phi = None
                        print("Completed GoDropLoc3 state! Moving to next state GoPickHome")
                        self.state = RobotStateEnum.GoPickHome
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoDropLoc state failed! Aborting and reverting to StayIdle")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.drop3_home_pose)
                        pass
                    else:
                        print("GoDropLoc3 state node not activated!")
                        pass

                # publish state 
                current_state = RobotState()
                current_state.robot_state = self.state.value
                self.state_publisher.publish(current_state)

                rospy.sleep(0.1)
            
        elif (user_input == "no"):
            return

        else:
            print("Please enter valid input :) !")
            return

        