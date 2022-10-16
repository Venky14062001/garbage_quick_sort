#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import sys
import rospy
import getch
from std_srvs.srv import SetBool, SetBoolRequest
from detection_msgs.srv import yolo, yoloRequest

def client():
	rospy.init_node('client')
	rospy.wait_for_service('detection_service')
	# string_client = rospy.ServiceProxy('detection_service', SetBool)

	xy_client = rospy.ServiceProxy('detection_service', yolo)

	z = 0.15
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		k = ord(getch.getch())
		if k == ord('q'):
			print('Keep quiet...')
			response = xy_client(z)
			#response = string_client(False)
			print(response.x, response.y, response.success)
		# elif k == ord('w'):
		# 	continue
		# 	print('Publishing data...')
		# 	response = string_client(True)
		# 	print(response)
		else:
			pass
		#return response.success, response.message


if __name__ == '__main__':
    #print("Response: [%s] %s" % (client()))
	client()
