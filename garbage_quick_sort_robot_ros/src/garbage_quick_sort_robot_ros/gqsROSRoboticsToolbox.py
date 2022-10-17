#!/usr/bin/env python3
import sys
import numpy as np

import rospy
from rospy import Duration
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory

from garbage_quick_sort_robot_msg.msg import EffectorPose, RobotState
from garbage_quick_sort_robot_msg.srv import RobotStateFbk, RobotStateFbkResponse, EffectorPoseFbk, EffectorPoseFbkResponse
from garbage_quick_sort_robot_ik.gqsIK import GarbageQuickSortRobotIK

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Bool

from roboticstoolbox import jtraj
import copy

'''This class acts as a ROS Node to interface with Dynamixel Motors and MoveIt!, subscribes to joint state and publishes trajectory when joint goal is received'''

class GarbageQuickSortRobotROSRoboticsToolbox:
    def __init__(self):
        rospy.init_node("GarbageQuickSortROS", anonymous=True)

        # robot IK solver
        self.iksolver = GarbageQuickSortRobotIK()

        # MoveIt! setup
        # moveit_commander.roscpp_initialize(sys.argv)
        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()

        # self.group_name = "garbage_quick_sort_robot_arm"
        # self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # self.planning_frame = self.move_group.get_planning_frame()
        # self.eef_link = self.move_group.get_end_effector_link()

        # self.group_names = self.robot.get_group_names()

        # dynamixel joint setup
        # store the joint limit values for the motor
        self.dynamixel_joint_limit_lower = np.array(
            [-1.5*np.pi, -1.5*np.pi, -1.5*np.pi, -1.5*np.pi])
        self.dynamixel_joint_limit_upper = np.array(
            [1.5*np.pi, 1.5*np.pi, 1.5*np.pi, 1.5*np.pi])

        # store the current joint state of the robot (the position is with respect to global frame (with respect to previous joint))
        self.joint_state_pos = None
        self.joint_state_vel = None
        self.joint_state_eff = None

        self.current_goal = None

        self.joint_names = np.array(
            ["joint_1", "joint_2", "joint_3", "joint_4"])

        # store the pose of the robot after every successful pose movement
        self.robot_pose = None

        # check if goal is commanded
        self.goal_commanded = False
        self.goal_tolerance = np.array([0.12, 0.12, 0.12, 0.12])
        # 0 is no goal received, 1 is in progress, 2 is failure, 3 is success
        self.reached_goal = 0
        self.goal_receive_time = None
        self.reach_goal_timeout = 60  # seconds

        # an attempt to solve the constant offset
        # offset assumed to be proportional to torque experienced (COM calculated)
        # self.home_offset = np.array([-0.18, 0.25, 0.20, 0.08]) # with pump exist
        self.home_offset = np.array([-0.13, 0.16, 0.18, 0.00]) # without pump
        self.ik_soln_exists = False
        self.traj_success = False

        # time to cover 1rad angle (based on max angle to cover)
        self.time_per_rad = 1.5
        self.time_steps_per_sec = 5

        # monitor if need to be activated
        self.active = False
        # added for test cases
        self.active_global_states = [1, 3, 4, 6, 7, 8]  

        # state number 6, start move down is a special state 
        self.special_state = 4
        self.global_state = None

        # store the suction state
        self.suction_state = False

        # visualize trajectory in RViz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory, queue_size=10
        )

        # publisher to the dynamixel trajectory topic
        self.joint_goal_publisher = rospy.Publisher(
            "/dynamixel_workbench/joint_trajectory",
            JointTrajectory, queue_size=1
        )

        # subscribe to the current joint states of the robot
        self.joint_state_subscriber = rospy.Subscriber(
            "/dynamixel_workbench/joint_states", JointState,
            self.joint_state_callback
        )

        # subscribe to the suction state (IMPORTANT to return state succeded as soon as the suction is activated)
        self.suction_state_subscriber = rospy.Subscriber(
            "/garbage_quick_sort/arduino/suction_active", Bool,
            self.suction_state_callback
        )

        # create the required subscriber to get the commanded end effector pose from higher packages
        self.end_effector_pose_subscriber = rospy.Subscriber(
            "/garbage_quick_sort/end_effector_pose", EffectorPose,
            self.pose_goal_callback
        )

        # subscribe to global state to know when to get activated
        self.global_state_subscriber = rospy.Subscriber(
            "/garbage_quick_sort/global_state", RobotState,
            self.global_state_callback
        )

        # setup server to provide pose feedback (queried for pose by other packages after a goal is successful)
        self.pose_robot_server = rospy.Service(
            "/garbage_quick_sort/effector_pose_service", EffectorPoseFbk,
            self.effector_pose_server_callback)

        # setup server to respond to state machine
        self.go_robot_server = rospy.Service(
            "/garbage_quick_sort/go_robot_service", RobotStateFbk,
            self.go_robot_server_callback)

        print("GQS ROS Initialized!")

    # update the global state of robot
    def global_state_callback(self, global_state):
        if (global_state.robot_state in self.active_global_states):
            self.active = True
            self.global_state = global_state.robot_state
            
        else:
            self.active = False
            # reset goal status values
            self.goal_commanded = False
            self.reached_goal = 0
            self.current_goal = None
            self.goal_receive_time = None

    # update the suction state 
    def suction_state_callback(self, suction_msg):
        self.suction_state = suction_msg.data

    # server to respond to effector pose query
    def effector_pose_server_callback(self, req):
        res = EffectorPoseFbkResponse()
        # send 0 from this node for type (not relevant), to be filled by higher packages
        res.type = 0
        res.pose_value = self.robot_pose

        return res

    # server to respond to feedback of goal
    def go_robot_server_callback(self, req):
        res = RobotStateFbkResponse()
        res.active_status = self.active
        res.goal_status = self.reached_goal

        return res

    # update joint state (responsible for updating reached_goal if active)
    def joint_state_callback(self, current_state):
        # ensure all joints are zeroed properly and the axis of rotation is correct
        self.joint_names = np.array(current_state.name)
        self.joint_state_pos = np.array(current_state.position)
        self.joint_state_vel = np.array(current_state.velocity)
        self.joint_state_eff = np.array(current_state.effort)

        if self.active:
            # if goal is commanded, check if it has reached within tolerance
            if self.goal_commanded:
                # first check if it special state, if yes, then keep checking force sensor readings, if force sensor detects something, then complete state
                if (self.global_state == self.special_state):
                    # if suction is activated, goal completed
                    if self.suction_state:
                        self.reached_goal = 3
                        self.robot_pose = None
                    elif np.all(np.less_equal(np.abs(self.joint_state_pos - self.current_goal), self.goal_tolerance)):
                        self.reached_goal = 3
                    # check if goal timeout
                    elif (rospy.Time.now().secs - self.goal_receive_time.secs) > self.reach_goal_timeout:
                        rospy.logerr(
                            "Goal timeout reached! Robot not reached goal state!")
                        self.reached_goal = 2
                        # reupdate robot_pose to None if failed
                        self.robot_pose = None
                    else:
                        # we can assume this state means goal is in progress (dont know if there is a better way?)
                        self.reached_goal = 1
                elif (self.ik_soln_exists and self.traj_success):
                    if np.all(np.less_equal(np.abs(self.joint_state_pos - self.current_goal), self.goal_tolerance)):
                        self.reached_goal = 3
                    # check if goal timeout
                    elif (rospy.Time.now().secs - self.goal_receive_time.secs) > self.reach_goal_timeout:
                        rospy.logerr(
                            "Goal timeout reached! Robot not reached goal state!")
                        self.reached_goal = 2
                        # reupdate robot_pose to None if failed
                        self.robot_pose = None
                    else:
                        # we can assume this state means goal is in progress (dont know if there is a better way?)
                        self.reached_goal = 1
                # this case occurs if goal is commanded, but no soln exists, then return failure
                else:
                    self.reached_goal = 2
            # this state means goal is not received yet, so inform corresponding nodes to publish goal again
            else:
                self.reached_goal = 0
                # reupdate robot_pose to None if no goal yet
                self.robot_pose = None
        else:
            pass

    # return 0 if both not valid, 1 if ED valid, 2 if EU valid, 3 if both valid
    def check_joint_limits(self, joint_soln_ik_frame):
        EU_soln = joint_soln_ik_frame["EU"]
        ED_soln = joint_soln_ik_frame["ED"]

        EU_soln_valid = np.all(np.greater_equal(EU_soln, self.dynamixel_joint_limit_lower)) and np.all(
            np.less_equal(EU_soln, self.dynamixel_joint_limit_upper))
        ED_soln_valid = np.all(np.greater_equal(ED_soln, self.dynamixel_joint_limit_lower)) and np.all(
            np.less_equal(ED_soln, self.dynamixel_joint_limit_upper))

        if (EU_soln_valid) and (ED_soln_valid):
            return 3
        elif (EU_soln_valid) and (not ED_soln_valid):
            return 2
        elif (not EU_soln_valid) and (ED_soln_valid):
            return 1
        else:
            return 0

    # this function creates the required trajectory message based on current state and goal
    def create_trajectory_msg(self, joint_motor_frame):

        # find the difference between joint states
        diff_joint_states = np.abs(joint_motor_frame - self.joint_state_pos)
        # find the max angle from the diff
        max_joint_angle = np.max(diff_joint_states)

        # find the time to go to the angle
        traj_time = round(max_joint_angle * self.time_per_rad)

        traj_msg = JointTrajectory()
        head = Header()
        head.stamp = rospy.Time.now()
        traj_msg.header = head
        traj_msg.joint_names = self.joint_names

        # create a time vector
        time_vec = np.linspace(0, traj_time, round(self.time_steps_per_sec * traj_time))

        # use robotics toolbox to compute trajectory
        try:
            gen_traj = jtraj(self.joint_state_pos, joint_motor_frame, time_vec)
        except:
            return False, None

        for i in range(len(gen_traj)):
            tmp_traj_pt = JointTrajectoryPoint()

            tmp_traj_pt.positions = gen_traj.s[i]
            tmp_traj_pt.velocities = gen_traj.sd[i]
            tmp_traj_pt.accelerations = gen_traj.sdd[i]

            tmp_traj_pt.time_from_start = Duration(time_vec[i])

            traj_msg.points.append(tmp_traj_pt)

        return True, traj_msg

    # if a new goal pose is received, calculate joint angles, update state and call the function to publish goal
    # responsible for updating goal_commanded
    def pose_goal_callback(self, pose):
        # only send goal if there is no current goal in execution
        if self.active:
            if self.goal_commanded:
                print("A goal is already in execution! Aborting the latest send goal to complete the existing one!")
                return

            else:
                req_pose = np.array([pose.x, pose.y, pose.z, pose.phi])

                ik_joint_sol = self.iksolver.get_joint_soln(req_pose)

                # check if joint soln is within limits
                joint_valid_check = self.check_joint_limits(ik_joint_sol)

                # choose the solution based on output (prefer EU config)
                if (joint_valid_check == 3):
                    sel_ik_joint_sol = ik_joint_sol["EU"]
                    self.ik_soln_exists = True
                elif (joint_valid_check == 2):
                    sel_ik_joint_sol = ik_joint_sol["EU"]
                    self.ik_soln_exists = True
                elif (joint_valid_check == 1):
                    sel_ik_joint_sol = ik_joint_sol["ED"]
                    self.ik_soln_exists = True
                else:
                    rospy.logerr("No valid joint IK solution found! Aborting pose goal request")
                    self.ik_soln_exists = False
                    self.goal_commanded = True
                    return

                # The printed solution angles are with respect to global frame (XYZ), joint type soln
                print("Calculated joint solution is: ")
                self.iksolver.print_joint_deg(sel_ik_joint_sol)

                # get robot current state, make MoveIt go to that state first then use toolbox to get traj and pass that traj to MoveIt!
                # moveit_start_joint_vals = self.move_group.get_current_joint_values()
                # moveit_start_joint_vals[0] = self.joint_state_pos[0]
                # moveit_start_joint_vals[1] = self.joint_state_pos[1]
                # moveit_start_joint_vals[2] = self.joint_state_pos[2]
                # moveit_start_joint_vals[3] = self.joint_state_pos[3]

                # # get MoveIt! to actual motor state
                # self.move_group.go(moveit_start_joint_vals, wait=True)
                # self.move_group.stop()
                # rospy.sleep(1.0)

                # # get state again and plan to goal state
                # moveit_goal_joint_vals = self.move_group.get_current_joint_values()
                # moveit_goal_joint_vals[0] = sel_ik_joint_sol[0]
                # moveit_goal_joint_vals[1] = sel_ik_joint_sol[1]
                # moveit_goal_joint_vals[2] = sel_ik_joint_sol[2]
                # moveit_goal_joint_vals[3] = sel_ik_joint_sol[3]

                # # when generating trajectory, try to change timestamps
                # self.move_group.set_joint_value_target(moveit_goal_joint_vals)
                # self.move_group.plan()

                # add offset to joint_goal
                sel_ik_joint_sol_off = copy.deepcopy(sel_ik_joint_sol)

                sel_ik_joint_sol_off[0] = - sel_ik_joint_sol[0] + self.home_offset[0] # the negative is to counter the gear change in direction
                sel_ik_joint_sol_off[1] = sel_ik_joint_sol[1] + self.home_offset[1]
                sel_ik_joint_sol_off[2] = sel_ik_joint_sol[2] + self.home_offset[2]
                sel_ik_joint_sol_off[3] = sel_ik_joint_sol[3] + self.home_offset[3]

                sel_ik_joint_sol_off = np.array(sel_ik_joint_sol_off)
                revise_plan = self.create_trajectory_msg(sel_ik_joint_sol_off)

                # check if planning succeeded
                if (revise_plan[0]):
                    self.traj_success = True
                else:
                    print("Toolbox unable to plan trajectory!")
                    self.goal_commanded = True
                    self.traj_success = False
                    return

                self.joint_goal_publisher.publish(revise_plan[1])

                # update goal status
                self.goal_commanded = True
                self.current_goal = sel_ik_joint_sol_off
                self.goal_receive_time = rospy.Time.now()

                self.robot_pose = pose

                # also publish to RViz for visualization
                # self.move_group.go(wait=True)
                # self.move_group.stop()

        else:
            print("Joint goal received when not active! Ignoring request!")
            return
