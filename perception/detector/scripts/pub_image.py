#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Float32

cap = cv2.VideoCapture(0)
print(cap.isOpened())
bridge = CvBridge()

def talker():
	pub = rospy.Publisher('/garbage_quick_sort/camera/image', Image, queue_size = 10)
	rospy.init_node('image', anonymous = False)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		ret, frame = cap.read()
		if not ret:
			break
		msg = bridge.cv2_to_imgmsg(frame, "bgr8")
		pub.publish(msg)
		if cv2.waitKey(10) & 0xFF == ord('q'):
			break
		if rospy.is_shutdown():
			cap.release()
			cv2.destroyAllWindows()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

