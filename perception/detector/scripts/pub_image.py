#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from detector.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Float32


cap = cv2.VideoCapture("test.mp4")
#"file_example_MP4_480_1_5MG.mp4"
print(cap.isOpened())
bridge = CvBridge()

'''
def callback(BoundingBoxes):
	rospy.loginfo("I heard", BoundingBoxes)
'''

def talker():
	pub = rospy.Publisher('/zedm/zed_node/left/image_rect_color', Image, queue_size = 10)
	rospy.init_node('image', anonymous = False)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		#sub = rospy.Subscriber('/boundingbox', BoundingBoxes, callback)
		ret, frame = cap.read()
		if not ret:
			break
		msg = bridge.cv2_to_imgmsg(frame, "bgr8")
		pub.publish(msg)
		#cv2.imshow("frame",frame)
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

