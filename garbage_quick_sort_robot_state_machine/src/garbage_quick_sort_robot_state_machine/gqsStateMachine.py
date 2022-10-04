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

class GarbageQuickSortRobotStateMachine:
    def __init__(self):
        rospy.init_node("GarbageQuickSortStateMachine", anonymous=True)
        self.state = RobotStateEnum.GoRestPose

        # define predefined poses here
        self.rest_pose = EffectorPose()
        self.rest_pose.x = 0.359
        self.rest_pose.y = 0.000
        self.rest_pose.z = 0.473
        self.rest_pose.phi = 0.175

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

        # create client handlers for GetLoc, StartSuction and StartRelease as well

        print("Garbage Quick Sort state machine activated :) !")

    def run(self):
        # add a statement to get input on whether to start program
        user_input = input("Ready to get moving..? ('yes' or 'no')")

        if (user_input == "yes"):
            while not rospy.is_shutdown():
                # conditional statements to decide and execute certain services
                if (self.state == RobotStateEnum.StayIdle):
                    pass
                if (self.state == RobotStateEnum.GoRestPose):
                    req = RobotStateFbkRequest()
                    try:
                        go_robot_fbk = self.go_robot_client_obj(req)
                    except rospy.ServiceException as e:
                        print("Service call GoRestPose failed: ", e)
                        continue
                    
                    # examine response to see if task succeded, if yes, switch state, if no, abort and GoRestPose
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 3)):
                        print("Completed GoRestPose state! Changing to StayIdle state now")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 2)):
                        print("GoRestPose state failed! Manual intervention required")
                        self.state = RobotStateEnum.StayIdle
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        self.pose_publisher.publish(self.rest_pose)
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
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == True)):
                        print("Completed GoPickHome state! Moving to next state GetPickLoc")
                        self.state = RobotStateEnum.GetPickUpLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == False)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.GoRestPose
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
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == True)):
                        print("Completed GoPickUpLoc state! Moving to next state StartSuction")
                        self.state = RobotStateEnum.StartSuction
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == False)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.GoRestPose
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
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == True)):
                        print("Completed GoDropHome state! Moving to next state GetDropLoc")
                        self.state = RobotStateEnum.GetDropLoc
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == False)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.GoRestPose
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
                    if ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == True)):
                        print("Completed GoDropLoc state! Moving to next state StartRelease")
                        self.state = RobotStateEnum.StartRelease
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == False)):
                        print("GoPickHome state failed! Aborting and reverting to GoRestPose")
                        self.state = RobotStateEnum.GoRestPose
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 1)):
                        pass
                    elif ((go_robot_fbk.active_status == True) and (go_robot_fbk.goal_status == 0)):
                        # republish goal state
                        pass
                    else:
                        print("GoDropLoc state node not activated!")
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

        