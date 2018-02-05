#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

"""
This program will take an image from the color camera and will
threshold the image to show only red elements. This behaviour can 
be changes by changing the constant values of the threshold limits.
"""

HUE_LOW = 0
HUE_HIGH = 179
SAT_LOW = 120
SAT_HIGH = 255
VAL_LOW = 170
VAL_HIGH = 255

class wall_follow_cam():
	def __init__(self):
		self.started = False
		self.to_check = True
		self.bridge = CvBridge()
		#upper and lower bound arrays for thresholding bgr image
		self.lowerB = np.array([HUE_LOW, SAT_LOW, VAL_LOW])
		self.upperB = np.array([HUE_HIGH, SAT_HIGH, VAL_HIGH])
		
		self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.thresholdBGR, queue_size=1)
		self.pub = rospy.Publisher('/camera/my_img/thresholded', Image, queue_size=1)



		
	def thresholdBGR(self, msg):
		if not self.started:
			rospy.loginfo("Thresholding of feed from the camera has started")
			self.started = True
		if not self.to_check:
			self.to_check = True
			return
		else:
			self.to_check = False	

		#convert the received message to bgr 8UC3
		frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		height, width = frame.shape[:2]
		frame = frame [8*(height/10):, :]
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		
		frame = cv2.medianBlur(frame, 5)
		frame = cv2.erode(frame, np.ones((3,3)))
		frame = cv2.dilate(frame, np.ones((3,3)))
		
		frame = cv2.inRange(frame, self.lowerB, self.upperB)
		frame = cv2.erode(frame, np.ones((5,5)))
		frame = cv2.dilate(frame, np.ones((5,5)))
		
		#Convert to ros image type to by published
		frame = self.bridge.cv2_to_imgmsg(frame, "mono8")
		self.pub.publish(frame)
	
if __name__ == '__main__':
	rospy.init_node("wall_follow_cam")
	wall_follow_cam()
	rospy.spin()

