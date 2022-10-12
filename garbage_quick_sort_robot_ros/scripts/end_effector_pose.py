#!/usr/bin/env python3

'''use this script to get the end effector (x, y, z, phi) in global frame using (link_4 x, y, z, phi, yaw) as i/p'''

import numpy as np

def deg2rad(val):
    return (val*np.pi)/180

# change these two if robot link lengths change
link4_length = 0.159
zOffset = 0.131 

# Obtain all these from RViz
phi = -90 # this is essentially sum of joint 2, 3, 4
x = 0.328487
y = 0.0
z = 0.25055
yaw = 0 # value of joint 1

yaw_rad = deg2rad(yaw)
phi_rad = deg2rad(phi)

ylocal = z - zOffset
xlocal = np.sqrt(np.square(x) + np.square(y))

# create a homogenous transform matrix for end effector
hom_mat = np.array([[np.cos(phi_rad), -np.sin(phi_rad), xlocal], [np.sin(phi_rad), np.cos(phi_rad), ylocal], [0, 0, 1]])
local_coord = np.matmul(hom_mat, np.array([[link4_length], [0], [1]]))

# transform local coordinates back to global coordinates
global_z = local_coord[1] + zOffset

global_x_1 = local_coord[0] * np.cos(yaw_rad)
global_x_2 = - local_coord[0] * np.cos(yaw_rad)

global_y_1 = np.tan(yaw_rad) * global_x_1
global_y_2 = np.tan(yaw_rad) * global_x_2

# Lets PRINT!
print("End Effector Pose 1: {0:.3f}  {1:.3f}  {2:.3f}  {3:.3f}".format(global_x_1[0], global_y_1[0], global_z[0], phi_rad))
print("End Effector Pose 2: {0:.3f}  {1:.3f}  {2:.3f}  {3:.3f}".format(global_x_2[0], global_y_2[0], global_z[0], phi_rad))



