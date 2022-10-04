#!/usr/bin/env python3
import numpy as np

'''This class is an inverse kinematic solver for the requested end effector pose'''

class GarbageQuickSortRobotIK:
    def __init__(self):
        self.nDOF = 4
        self.link_lengths = [0.1314, 0.236, 0.186, 0.1]
        # the z offset (z-location of joint 2)  
        self.zOffset = self.link_lengths[0]

        # the input for inverse kinematic soln's or any pose is 
        # given in the format (x, y, z, phi), x, y, z is wrt global frame (in meters)
        # phi is the angle of air suction wrt local frame (in degrees)

        # for calculating the location to go to pick the object
        # the x,y location is from software, get the z distance using ultrasonic 
        # software end also get ratio of pick up box side lengths to the x-y location 
        # use the ratio to command actual x-y difference from home position

    # return IK soln for the four joints (in m, rad)
    # input end effector pose: x, y, z, phi (m, m, m, rad)
    def get_joint_soln(self, end_effector_pose):
        # calculate the yaw angle
        yaw = np.arctan2(end_effector_pose[1], end_effector_pose[0]) # joint 0
        joint_0 = yaw

        # recalculate end effector pose in local 2D frame
        ye = end_effector_pose[2] - self.zOffset
        xe = np.sqrt(np.square(end_effector_pose[0]) + np.square(end_effector_pose[1]))

        # calculate wrist positions
        xw = xe - (self.link_lengths[3] * np.cos(end_effector_pose[3]))
        yw = ye - (self.link_lengths[3] * np.sin(end_effector_pose[3]))
    
        # calculate alpha
        alpha = np.arctan2(yw, xw)

        # calculate beta 
        beta = np.arccos((np.square(self.link_lengths[1]) + np.square(self.link_lengths[2]) - np.square(xw) - np.square(yw)) / (2 * self.link_lengths[1] * self.link_lengths[2]))

        # calculate joint angle 2
        joint_2_ed = np.pi - beta

        # calculate gamma
        gamma = np.arccos((np.square(xw) + np.square(yw) + np.square(self.link_lengths[1]) - np.square(self.link_lengths[2])) / (2 * self.link_lengths[1] * np.sqrt(np.square(xw) + np.square(yw))) )

        # calculate joint angle 1
        joint_1_ed = alpha - gamma

        # calculate joint angle 3
        joint_3_ed = end_effector_pose[3] - joint_1_ed - joint_2_ed

        # calculate joint angles for elbow up configuration
        joint_1_eu = joint_1_ed + (2 * gamma)
        joint_2_eu = - joint_2_ed
        joint_3_eu = joint_3_ed + (2 * joint_2_ed) - (2 * gamma)

        return {"EU": np.array([joint_0, joint_1_eu, joint_2_eu, joint_3_eu]), "ED": np.array([joint_0, joint_1_ed, joint_2_ed, joint_3_ed])}

    # utility functions
    def rad2deg(self, val):
        return (val*180)/np.pi

    def deg2rad(self, val):
        return (val*np.pi)/180

    def print_joint_deg(self, val):
        joint_deg = self.rad2deg(val)

        print("Joint 0: {0:.2f}    Joint 1: {1:.2f}    Joint 2: {2:.2f}    Joint 3: {3:.2f}".format(joint_deg[0], joint_deg[1], joint_deg[2], joint_deg[3]))