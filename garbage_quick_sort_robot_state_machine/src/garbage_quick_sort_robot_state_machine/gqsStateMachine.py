#!/usr/bin/env python3

'''This class acts as the state machine for the GQS Robot, instructing other nodes on what state robot is in and getting feedback from concerned nodes'''

import rospy
import enum
from garbage_quick_sort_robot_msg.msg import RobotState, EffectorPose
from garbage_quick_sort_robot_msg.srv import RobotStateFbk, RobotStateFbkRequest, RobotStateFbkResponse

class RobotStateEnum(enum.Enum):
    GoRestPose = 0
    GoPickHome = 1
    GetPickUpLoc = 2
    GoPickUpLoc = 3
    StartSuction = 4
    GoDropHome = 5
    GetDropLoc = 6
    GoDropLoc = 7
    StartRelease = 8
    StayIdle = 9
    TransitionRestPose = 10 # this state is needed to transition from any motor state to abort state, to inform motor of this state change
    Pos0 = 18
    Trans1 = 19
    Pos1 = 11   # test cases
    Trans2 = 15
    Pos2 = 12
    Trans3 = 16
    Pos3 = 13
    Trans4 = 17
    Pos4 = 14

    # Go Home
    # Trans
    # Go Pick Home
    # Trans
    # Get Pick Loc
    # 

class GarbageQuickSortRobotStateMachine:
    def __init__(self):
        rospy.init_node("GarbageQuickSortStateMachine", anonymous=True)
        self.state = RobotStateEnum.Pos1

        # define predefined poses here
        # Joints 0, 0, 0, 0     0, 0, 0, 0
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
        self.pose_3.x = 0.506
        self.pose_3.y = 0.000
        self.pose_3.z = 0.103
        self.pose_3.phi = -0.175

        # Joints 0, 40, -20, -60     0, 0.698132, -0.349066, -1.0472
        self.pose_4 = EffectorPose()
        self.pose_4.x = 0.439
        self.pose_4.y = 0.000
        self.pose_4.z = 0.212
        self.pose_4.phi = -0.698

        # a bunch of poses to try out
        # [0.464, 0.0, 0.332, 0.0]
        # [0.471, 0.0, 0.231, 0.873]
        # [0.451, 0.0, 0.265, -0.524]
        # create global state publisher
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

        # server for YOLO inference
        # self.yolo_server = rospy.Service("/garbage_quick_sort/z_service", 
        #                         self.yolo_service_callback, )


        # create client handlers for GetLoc, StartSuction and StartRelease as well
        # rospy.wait_for_service("/garbage_quick_sort/response_RobotStateFbk")
        # try:
        #     self.go_robot_client_obj = rospy.ServiceProxy("/garbage_quick_sort/response_RobotStateFbk", RobotStateFbk)
        # except rospy.ServiceException as e:
        #     print("YOLO Robot service object failed: ", e)

        print("Garbage Quick Sort state machine activated :) !")

    def run(self):
        # add a statement to get input on whether to start program
        user_input = input("Ready to get moving..? ('yes' or 'no')")

        if (user_input == "yes"):
            while not rospy.is_shutdown():
                # conditional statements to decide and execute certain services
                if (self.state == RobotStateEnum.StayIdle):
                    pass

                elif (self.state == RobotStateEnum.TransitionRestPose): 
                    # just need to go to RestPose here
                    self.state = RobotStateEnum.GoRestPose

                elif (self.state == RobotStateEnum.GoRestPose):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoRestPose failed: ", e)
                        continue
                    
                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoRestPose state! Changing to StayIdle state now")
                        self.state = self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoRestPose state failed! Manual intervention required")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        # self.pose_publisher.publish(self.rest_pose)
                        pass
                    else:
                        print("GoRestPose state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoPickHome):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoPickHome  failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoPickHome state! Moving to next state GetPickLoc")
                        self.state = self.state = RobotStateEnum.GetPickUpLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        # self.pose_publisher.publish(self.home_pose)
                        pass
                    else:
                        print("GoHomePose state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoPickUpLoc):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoPickUpLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoPickUpLoc state! Moving to next state StartSuction")
                        self.state = RobotStateEnum.StartSuction
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        pass
                    else:
                        print("GoPickUpLoc state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoDropHome):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropHome failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoDropHome state! Moving to next state GetDropLoc")
                        self.state = RobotStateEnum.GetDropLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        pass
                    else:
                        print("GoDropHome state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.GoDropLoc):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoDropLoc state! Moving to next state StartRelease")
                        self.state = RobotStateEnum.StartRelease
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        pass
                    else:
                        print("GoDropLoc state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.Pos0):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoDropLoc state! Moving to next state StartRelease")
                        self.state = RobotStateEnum.Trans1
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.home_pose)
                        pass
                    else:
                        print("GoDropLoc state node not activated!")
                        pass

                # Test cases
                elif (self.state == RobotStateEnum.Pos1):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoDropLoc state! Moving to next state StartRelease")
                        self.state = RobotStateEnum.Trans2
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.pose_1)
                        pass
                    else:
                        print("GoDropLoc state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.Pos2):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoDropLoc state! Moving to next state StartRelease")
                        self.state = RobotStateEnum.StayIdle #RobotStateEnum.Trans3
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.pose_2)
                        pass
                    else:
                        print("GoDropLoc state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.Pos3):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoDropLoc state! Moving to next state StartRelease")
                        self.state = RobotStateEnum.Trans4
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.pose_3)
                        pass
                    else:
                        print("GoDropLoc state node not activated!")
                        pass

                elif (self.state == RobotStateEnum.Pos4):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoDropLoc failed: ", e)
                        continue

                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoDropLoc state! Moving to next state StartRelease")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.TransitionRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.pose_4)
                        pass
                    else:
                        print("GoDropLoc state node not activated!")
                        pass
                
                elif (self.state == RobotStateEnum.Trans1):
                    self.state = RobotStateEnum.Pos1

                elif (self.state == RobotStateEnum.Trans2):
                    self.state = RobotStateEnum.Pos2

                elif (self.state == RobotStateEnum.Trans3):
                    self.state = RobotStateEnum.Pos3

                elif (self.state == RobotStateEnum.Trans4):
                    self.state = RobotStateEnum.Pos4

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

        