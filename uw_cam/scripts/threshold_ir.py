#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class wall_follow_cam():
	def __init__(self):
		self.started = False
		self.to_check = True
		#will be used to convert between OpenCV and ros Images
                self.bridge = CvBridge()
		
		#Subscribe to the raw color images from the camera
		self.sub = rospy.Subscriber('/camera/ir/image_raw', Image, self.thresholdIR, queue_size=1)
		#The color image will be thresholded and published here
		self.pub = rospy.Publisher('/camera/my_img/thresholded', Image, queue_size=1)

		
	def thresholdIR(self, msg):
		if not self.started:
			rospy.loginfo("Thresholding of feed from the IR camera has started")
			self.started = True
		if not self.to_check:
			self.to_check = True
			return
		else:
			self.to_check = False	
		#convert the received message to bgr 8UC3
		frame = self.bridge.imgmsg_to_cv2(msg, "8UC1")
		height, width = frame.shape[:2]
		frame = frame [7*(height/10):, :]
		#apply median blur
		frame = cv2.medianBlur(frame, 5)
		#Perform the required morphological operations
		frame = cv2.erode(frame, np.ones((3,3)))
		frame = cv2.dilate(frame, np.ones((3,3)))
		#Threshold the image to the required bounds
		frame = cv2.inRange(frame, 170, 255)
		#morph the image to take out any unnecessary white pixels.
		frame = cv2.erode(frame, np.ones((2,2)))
		frame = cv2.dilate(frame, np.ones((2,2)))
		#Convert to ros image type to by published
		frame = self.bridge.cv2_to_imgmsg(frame, "mono8")
		self.pub.publish(frame)

if __name__ == '__main__':
	rospy.init_node("wall_follow_cam")
	wall_follow_cam()
	rospy.spin()

