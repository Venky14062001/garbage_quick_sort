#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import sys
import rospy
import getch
from std_srvs.srv import SetBool, SetBoolRequest

def client():
	rospy.init_node('client')
	rospy.wait_for_service('detection_service')
	string_client = rospy.ServiceProxy('detection_service', SetBool)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		k = ord(getch.getch())
		if k == ord('q'):
			print('Keep quiet...')
			response = string_client(False)
		elif k == ord('w'):
			print('Publishing data...')
			response = string_client(True)
		else:
			pass
		#return response.success, response.message


if __name__ == '__main__':
    #print("Response: [%s] %s" % (client()))
	client()