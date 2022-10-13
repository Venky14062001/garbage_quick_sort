#!/usr/bin/env python3
import rospy
from detector.detect import Detection

# Results
if __name__ == "__main__":
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(15)
    detection = Detection()
    try:
        while not rospy.is_shutdown():
            detection.inference() 
            rate.sleep()
    except (KeyboardInterrupt, StopIteration):
        pass

