#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import cv2
from cv_bridge import CvBridge, CvBridgeError

"""
This program will prevent the car from moving if the number
of white pixels in the thresholded image is less than MIN_PIXELS_TO_DRIVE.
"""

MIN_PIXELS_TO_DRIVE = 5

class safety():
	def __init__(self):
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber('/camera/my_img/thresholded', Image, self.tape_present, queue_size=1)
		self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size = 1)
	
	def tape_present(self, msg):
		frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
		white_pixels = cv2.countNonZero(frame)
		if white_pixels < MIN_PIXELS_TO_DRIVE:
			drive_msg_stamped = AckermannDriveStamped()
			drive_msg = AckermannDrive()
			drive_msg.speed = 0.0
			drive_msg.steering_angle = 0.0
			drive_msg.jerk = 0
			drive_msg.steering_angle_velocity = 0
			drive_msg_stamped.drive = drive_msg
			self.pub.publish(drive_msg_stamped)
		
if __name__ == '__main__':
	rospy.init_node("safety_cam")
	safety()
	rospy.spin()	
