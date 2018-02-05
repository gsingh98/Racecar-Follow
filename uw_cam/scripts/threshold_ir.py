#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

"""
This program will take an IR image and threshold it to show only 
the objects that glow in IR light.
"""

class wall_follow_cam():
	def __init__(self):
		self.started = False
		self.to_check = True
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber('/camera/ir/image_raw', Image, self.thresholdIR, queue_size=1)
		self.pub = rospy.Publisher('/camera/my_img/thresholded', Image, queue_size=1)

	#This will apply morphological operations to the image and then threshold and publish it	
	def thresholdIR(self, msg):
		if not self.started:
			rospy.loginfo("Thresholding of feed from the IR camera has started")
			self.started = True
		if not self.to_check:
			self.to_check = True
			return
		else:
			self.to_check = False	
		
		frame = self.bridge.imgmsg_to_cv2(msg, "8UC1")
		height, width = frame.shape[:2]
		
		frame = frame [7*(height/10):, :]
		frame = cv2.medianBlur(frame, 5)
		frame = cv2.erode(frame, np.ones((3,3)))
		frame = cv2.dilate(frame, np.ones((3,3)))
		
		frame = cv2.inRange(frame, 170, 255)
		frame = cv2.erode(frame, np.ones((2,2)))
		frame = cv2.dilate(frame, np.ones((2,2)))
		
		frame = self.bridge.cv2_to_imgmsg(frame, "mono8")
		self.pub.publish(frame)

if __name__ == '__main__':
	rospy.init_node("wall_follow_cam")
	wall_follow_cam()
	rospy.spin()

