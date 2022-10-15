#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, UInt8

#notes:
'''
import RobotStateFbk
Subscribe /garbage_qick_sort/global_state
when active state:
	return true/false in RobotStateFbk, active status and goal_status (1: )

'''
class ArduinoServiceHandler: 
	def __init__(self):
		rospy.Subscriber('/garbage_quick_sort/arduino/target_contact', Bool, self.target_contact_callback)

		self.pub_activate_suction = rospy.Publisher('/garbage_quick_sort/arduino/suction_state', Bool, queue_size=1) 
		self.activate_suction_msg = Bool()
		self.activate_suction_msg.data = False

		self.response_activate_suction = rospy.Service('/garbage_quick_sort/arduino/activate_suction', Trigger, self.activate_suction)

		self.suction_state = False
		self.target_distance = 0


	def target_distance_callback(self, msg):
		self.target_distance = msg.data

	def target_contact_callback(self, msg):
		self.contacted = msg.data

	def activate_suction(self, request):
		if request:
			self.activate_suction_msg.data = not self.activate_suction_msg.data
			return TriggerResponse(True, "suction pump triggered")

	def main(self):
		self.pub_activate_suction.publish(self.activate_suction_msg)



if __name__ == "__main__":
	rospy.init_node('detect', anonymous=True)
	arduino = ArduinoServiceHandler()
	rate = rospy.Rate(50)
	try:
		while not rospy.is_shutdown():
			arduino.main()
			rate.sleep()
	except (KeyboardInterrupt, StopIteration):
		pass


