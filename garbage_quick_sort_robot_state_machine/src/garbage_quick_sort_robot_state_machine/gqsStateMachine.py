#!/usr/bin/env python3

'''This class acts as the state machine for the GQS Robot, instructing other nodes on what state robot is in and getting feedback from concerned nodes'''

import rospy
import numpy as np
import enum
from garbage_quick_sort_robot_msg.msg import RobotState, EffectorPose
from garbage_quick_sort_robot_msg.srv import RobotStateFbk, RobotStateFbkRequest, RobotStateFbkResponse, EndEffectorPose, EndEffectorPoseRequest, EndEffectorPoseResponse

class RobotStateEnum(enum.Enum):
    GoHome = 0
    GoPickHome = 1
    GetPickUpLoc = 2
    GoPickUpLoc = 3
    StayIdle = 5

class GarbageQuickSortRobotStateMachine:
    def __init__(self):
        rospy.init_node("GarbageQuickSortStateMachine", anonymous=True)
        self.state = RobotStateEnum.GoHome

        # home pose
        self.home_pose = EffectorPose()
        self.home_pose.x = 0.53099
        self.home_pose.y = 0.000
        self.home_pose.z = 0.131
        self.home_pose.phi = 0.0

        self.pose_1 = EffectorPose()
        self.pose_1.x = 0.205
        self.pose_1.y = 0.02
        self.pose_1.z = 0.092
        self.pose_1.phi = -1.571

        self.pose_2 = EffectorPose()
        self.pose_2.x = 0.145
        self.pose_2.y = 0.27
        self.pose_2.z = 0.092
        self.pose_2.phi = -1.571

        # Joints 0, 20, -40, 10     0, 0.349066, -0.698132, 0.174533
        self.pose_3 = EffectorPose()
        self.pose_3.x = -0.013
        self.pose_3.y = 0.27
        self.pose_3.z = 0.092
        self.pose_3.phi = -1.571

        # Joints 0, 40, -20, -60     0, 0.698132, -0.349066, -1.0472
        self.pose_4 = EffectorPose()
        self.pose_4.x = -0.19
        self.pose_4.y = 0.27
        self.pose_4.z = 0.092
        self.pose_4.phi = -1.571
        
        # variables to store (x, y)
        self.inference_xy = EffectorPose()
        self.go_pick_up_loc = EffectorPose()

        self.state_publisher = rospy.Publisher(
            "/garbage_quick_sort/global_state", 
            RobotState, queue_size=10
        )

        # pose publisher
        self.pose_publisher = rospy.Publisher(
            "/garbage_quick_sort/end_effector_pose",
            EffectorPose, queue_size=1
        )

        # subscriber to get (x, y) location
        self.inference_subscriber = rospy.Subscriber(
            "/garbage_quick_sort/camera_frame/end_effector_pose",
            EffectorPose, self.inference_callback
        )

        # create the required client handlers
        rospy.wait_for_service("/garbage_quick_sort/go_robot_service")
        try:
            self.go_robot_client_obj = rospy.ServiceProxy("/garbage_quick_sort/go_robot_service", RobotStateFbk)
        except rospy.ServiceException as e:
            print("Go Robot service object failed: ", e)

        print(1)

        # create client handlers for GetLoc, StartSuction and StartRelease as well
        rospy.wait_for_service("/garbage_quick_sort/response_RobotStateFbk")
        try:
            self.inference_client = rospy.ServiceProxy("/garbage_quick_sort/response_RobotStateFbk", RobotStateFbk)
        except rospy.ServiceException as e:
            print("YOLO Robot service object failed: ", e)

        print(2)

        # create client handler for getting previous pose 
        rospy.wait_for_service("/garbage_quick_sort/end_effector_pose_service")
        try:
            self.end_effector_pose_client = rospy.ServiceProxy("/garbage_quick_sort/end_effector_pose_service", RobotStateFbk)
        except rospy.ServiceException as e:
            print("End effector pose service object failed: ", e)

        print("Garbage Quick Sort state machine activated :) !")

        print(3)

    def inference_callback(self, msg):
        self.inference_xy.x = msg.x
        self.inference_xy.y = msg.y
        self.inference_xy.z = msg.z
        self.inference_xy.phi = msg.phi

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

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoHome state! Moving to next state GetPickLoc")
                        self.state = self.state = RobotStateEnum.GoPickHome
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoHome state failed! Aborting and reverting to GoRestPose")
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

                elif (self.state == RobotStateEnum.GoPickHome):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoPickHome failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoPickHome state! Moving to next state GetPickLoc")
                        self.state = self.state = RobotStateEnum.GetPickUpLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.pose_1)
                        pass
                    else:
                        print("GoPickHome state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GetPickUpLoc):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GetPickLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GetPickUpLoc state! Moving to next state GoPickUpLoc")
                        self.state = self.state = RobotStateEnum.GoPickUpLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GetPickUpLoc state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        print("Waiting....")
                        pass
                    else:
                        print("GetPickUpLoc state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoPickUpLoc):
                    # get the effector pose currently
                    effector_pose_req = EndEffectorPoseRequest()
                    try:
                        end_effector_fbk = self.end_effector_pose_client(effector_pose_req)
                    except rospy.ServiceException as e:
                        print("Service call end effector pose failed: ", e)
                        continue

                    # use the effector pose to form transform matrix
                    transform_matrix = np.array([[np.cos(end_effector_fbk.phi), -np.sin(end_effector_fbk.phi), 0, end_effector_fbk.x], 
                                        [np.sin(end_effector_fbk.phi), np.cos(end_effector_fbk.phi), 0, end_effector_fbk.y],
                                        [0, 0, 1, end_effector_fbk.z],
                                        [0, 0, 0, 1]])
                    # transform to new location
                    camera_xyz = np.array([self.inference_xy[0], self.inference_xy[1], self.inference_xy[2]])
                    self.go_pick_up_loc = np.matmul(transform_matrix, camera_xyz)

                    print(self.go_pick_up_loc)

                    # req = RobotStateFbkRequest()
                    # try:
                    #     go_robot_fbk = self.go_robot_client_obj(req)
                    # except rospy.ServiceException as e:
                    #     print("Service call GoPickUpLoc failed: ", e)
                    #     continue

                    # # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    # if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                    #     print("Completed GoPickUpLoc state! Moving to next state StartSuction")
                    #     self.state = RobotStateEnum.StartSuction
                    # elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                    #     print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                    #     self.state = RobotStateEnum.TransitionRestPose
                    # elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                    #     pass
                    # elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                    #     # republish goal state
                    #     pass
                    # else:
                    #     print("GoPickUpLoc state node not activated!")
                    #     pass

                # elif (self.state == RobotStateEnum.GoDropHome):
                #     req = RobotStateFbkRequest()
                #     try:
                #         go_robot_fbk = self.go_robot_client_obj(req)
                #     except rospy.ServiceException as e:
                #         print("Service call GoDropHome failed: ", e)
                #         continue

                #     # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                #     if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                #         print("Completed GoDropHome state! Moving to next state GetDropLoc")
                #         self.state = RobotStateEnum.GetDropLoc
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                #         print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                #         self.state = RobotStateEnum.TransitionRestPose
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                #         pass
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                #         # republish goal state
                #         pass
                #     else:
                #         print("GoDropHome state node not activated!")
                #         pass

                # elif (self.state == RobotStateEnum.GoDropLoc):
                #     req = RobotStateFbkRequest()
                #     try:
                #         go_robot_fbk = self.go_robot_client_obj(req)
                #     except rospy.ServiceException as e:
                #         print("Service call GoDropLoc failed: ", e)
                #         continue

                #     # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                #     if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                #         print("Completed GoDropLoc state! Moving to next state StartRelease")
                #         self.state = RobotStateEnum.StartRelease
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                #         print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                #         self.state = RobotStateEnum.TransitionRestPose
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                #         pass
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                #         # republish goal state
                #         pass
                #     else:
                #         print("GoDropLoc state node not activated!")
                #         pass

                # elif (self.state == RobotStateEnum.Pos0):
                #     req = RobotStateFbkRequest()
                #     try:
                #         go_robot_fbk = self.go_robot_client_obj(req)
                #     except rospy.ServiceException as e:
                #         print("Service call GoDropLoc failed: ", e)
                #         continue

                #     # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                #     if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                #         print("Completed GoDropLoc state! Moving to next state StartRelease")
                #         self.state = RobotStateEnum.Trans1
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                #         print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                #         self.state = RobotStateEnum.TransitionRestPose
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                #         pass
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                #         # republish goal state
                #         self.pose_publisher.publish(self.home_pose)
                #         pass
                #     else:
                #         print("GoDropLoc state node not activated!")
                #         pass

                # # Test cases
                # elif (self.state == RobotStateEnum.Pos1):
                #     req = RobotStateFbkRequest()
                #     try:
                #         go_robot_fbk = self.go_robot_client_obj(req)
                #     except rospy.ServiceException as e:
                #         print("Service call GoDropLoc failed: ", e)
                #         continue

                #     # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                #     if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                #         print("Completed GoDropLoc state! Moving to next state StartRelease")
                #         self.state = RobotStateEnum.Trans2
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                #         print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                #         self.state = RobotStateEnum.TransitionRestPose
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                #         pass
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                #         # republish goal state
                #         self.pose_publisher.publish(self.pose_1)
                #         pass
                #     else:
                #         print("GoDropLoc state node not activated!")
                #         pass

                # elif (self.state == RobotStateEnum.Pos2):
                #     req = RobotStateFbkRequest()
                #     try:
                #         go_robot_fbk = self.go_robot_client_obj(req)
                #     except rospy.ServiceException as e:
                #         print("Service call GoDropLoc failed: ", e)
                #         continue

                #     # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                #     if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                #         print("Completed GoDropLoc state! Moving to next state StartRelease")
                #         self.state = RobotStateEnum.Trans3
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                #         print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                #         self.state = RobotStateEnum.TransitionRestPose
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                #         pass
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                #         # republish goal state
                #         self.pose_publisher.publish(self.pose_2)
                #         pass
                #     else:
                #         print("GoDropLoc state node not activated!")
                #         pass

                # elif (self.state == RobotStateEnum.Pos3):
                #     req = RobotStateFbkRequest()
                #     try:
                #         go_robot_fbk = self.go_robot_client_obj(req)
                #     except rospy.ServiceException as e:
                #         print("Service call GoDropLoc failed: ", e)
                #         continue

                #     # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                #     if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                #         print("Completed GoDropLoc state! Moving to next state StartRelease")
                #         self.state = RobotStateEnum.Trans4
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                #         print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                #         self.state = RobotStateEnum.TransitionRestPose
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                #         pass
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                #         # republish goal state
                #         self.pose_publisher.publish(self.pose_3)
                #         pass
                #     else:
                #         print("GoDropLoc state node not activated!")
                #         pass

                # elif (self.state == RobotStateEnum.Pos4):
                #     req = RobotStateFbkRequest()
                #     try:
                #         go_robot_fbk = self.go_robot_client_obj(req)
                #     except rospy.ServiceException as e:
                #         print("Service call GoDropLoc failed: ", e)
                #         continue

                #     # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                #     if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                #         print("Completed GoDropLoc state! Moving to next state StartRelease")
                #         self.state = RobotStateEnum.StayIdle
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                #         print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                #         self.state = RobotStateEnum.TransitionRestPose
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                #         pass
                #     elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                #         # republish goal state
                #         self.pose_publisher.publish(self.pose_4)
                #         pass
                #     else:
                #         print("GoDropLoc state node not activated!")
                #         pass
                
                # elif (self.state == RobotStateEnum.Trans1):
                #     self.state = RobotStateEnum.Pos1

                # elif (self.state == RobotStateEnum.Trans2):
                #     self.state = RobotStateEnum.Pos2

                # elif (self.state == RobotStateEnum.Trans3):
                #     self.state = RobotStateEnum.Pos3

                # elif (self.state == RobotStateEnum.Trans4):
                #     self.state = RobotStateEnum.Pos4

                # # publish state 
                # current_state = RobotState()
                # current_state.robot_state = self.state.value
                # self.state_publisher.publish(current_state)

                rospy.sleep(0.5)
            
        elif (user_input == "no"):
            return

        else:
            print("Please enter valid input :) !")
            return

        