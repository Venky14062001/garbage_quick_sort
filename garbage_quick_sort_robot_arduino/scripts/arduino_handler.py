#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, Int16, Float32
from garbage_quick_sort_robot_msg.msg import RobotState

import numpy as np

class ArduinoHandler: 
	def __init__(self):
		self.pub_activate_suction = rospy.Publisher('/garbage_quick_sort/arduino/suction_active', Bool, queue_size=1)
		# publiser for state machine to double confirm pickup of object
		self.pub_ultrasonic_mean = rospy.Publisher('/garbage_quick_sort/arduino/ultrasonic_mean', Float32, queue_size=1)

		# subscribe to global state
		self.global_state_sub = rospy.Subscriber('/garbage_quick_sort/global_state', RobotState, self.global_state_callback)
		self.force_sub = rospy.Subscriber('/garbage_quick_sort/arduino/distance', Int16, self.distance_msg_callback)

		self.force_roll_mean_size = 50
		self.force_roll_mean_buffer = np.zeros(self.force_roll_mean_size)
		self.roll_mean_index = 0

		self.activate_suction_msg = Bool()
		self.activate_suction_msg.data = False

		self.trigger_value = 5

		self.suction_active_states = [5, 6, 7, 8, 9, 10, 11]
		self.move_down_state = self.suction_active_states[0]
		self.start_global_state = False
		self.global_state = None
		# average distance value
		self.avg_ultrasonic = Float32() 

	def global_state_callback(self, state_msg):
		self.global_state = state_msg.robot_state
		self.start_global_state =True

	def avg_function(self):
		# takes the average of values in the buffer and triggers 
		avg_val = np.mean(self.force_roll_mean_buffer)
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
			# else:
			# 	self.activate_suction_msg.data = False
		else:
			self.activate_suction_msg.data = False

	def distance_msg_callback(self, msg):
		if self.roll_mean_index == self.force_roll_mean_size:
			# reset index to 0
			self.roll_mean_index = 0

		self.force_roll_mean_buffer[self.roll_mean_index] = msg.data

		# increase index
		self.roll_mean_index += 1

		# call averaging function 
		self.avg_function()

	def main(self):
		self.pub_activate_suction.publish(self.activate_suction_msg)
		self.pub_ultrasonic_mean.publish(self.avg_ultrasonic)

if __name__ == "__main__":
	rospy.init_node('arduino_handler', anonymous=True)
	arduino = ArduinoHandler()
	rate = rospy.Rate(50)
	try:
		while not rospy.is_shutdown():
			if arduino.start_global_state:
				arduino.main()
			else:
				pass
			rate.sleep()
	except (KeyboardInterrupt, StopIteration):
		pass


