#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, Int16, Float32
from garbage_quick_sort_robot_msg.msg import RobotState
from garbage_quick_sort_robot_msg.srv import UltrasonicMeanFbk, UltrasonicMeanFbkRequest, UltrasonicMeanFbkResponse

import numpy as np

class ArduinoHandler: 
	def __init__(self):
		rospy.init_node('arduino_handler', anonymous=True)

		self.pub_activate_suction = rospy.Publisher('/garbage_quick_sort/arduino/suction_active', Bool, queue_size=1)
		# create a server for state machine to query the ultrasonic distance to double confirm pickup of object
		self.ultrasonic_average_server = rospy.Service(
            "/garbage_quick_sort/arduino/ultrasonic_mean", UltrasonicMeanFbk,
            self.ultrasonic_mean_server_callback)

		# subscribe to global state
		self.global_state_sub = rospy.Subscriber('/garbage_quick_sort/global_state', RobotState, self.global_state_callback)
		self.ultrasonic_sensor_sub = rospy.Subscriber('/garbage_quick_sort/arduino/distance', Float32, self.distance_msg_callback)

		self.distance_roll_mean_size = 20
		self.distance_roll_mean_buffer = np.zeros(self.distance_roll_mean_size)
		self.roll_mean_index = 0

		self.activate_suction_msg = Bool()
		self.activate_suction_msg.data = False

		self.trigger_value = 3.5

		self.suction_active_states = [5, 6, 7, 8, 9, 10, 11]
		self.move_down_state = self.suction_active_states[0]
		self.start_global_state = False
		self.global_state = None
		# average distance value
		self.avg_ultrasonic = Float32() 

	def global_state_callback(self, state_msg):
		self.global_state = state_msg.robot_state
		self.start_global_state = True

	def ultrasonic_mean_server_callback(self, req):
		res = UltrasonicMeanFbkResponse()
		res.sensor_mean = self.avg_ultrasonic
		
		return res

	def avg_function(self):
		# takes the average of values in the buffer and triggers 
		avg_val = np.mean(self.distance_roll_mean_buffer)
		self.avg_ultrasonic.data = avg_val

		# if in move down state and suction not yet triggered, then trigger
		if ((self.global_state == self.move_down_state) and (not self.activate_suction_msg.data)):	
			if avg_val < self.trigger_value:
				self.activate_suction_msg.data = True
			else:
				self.activate_suction_msg.data = False
		elif self.global_state in self.suction_active_states:
			if self.activate_suction_msg.data:
				self.activate_suction_msg.data = True
			else:
				self.activate_suction_msg.data = False
		else:
			self.activate_suction_msg.data = False

		self.pub_activate_suction.publish(self.activate_suction_msg)

	def distance_msg_callback(self, msg):
		if self.roll_mean_index == self.distance_roll_mean_size:
			# reset index to 0
			self.roll_mean_index = 0

		self.distance_roll_mean_size[self.roll_mean_index] = msg.data

		# increase index
		self.roll_mean_index += 1

		# call averaging function 
		self.avg_function()
		



