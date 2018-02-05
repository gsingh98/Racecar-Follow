#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

"""
Author: Guramrit Singh

This program implements a line following algorithm for a racecar.
The car will be able to follow infrared tags using thresholded images
in which only the tape should appear white.
It will also decrease its speed at turns to avoid running off the line.
"""

LINES_TO_CHECK = 20
MAX_TURNING_ANGLE = 0.34
MAX_SPEED = 1.0
ROWS_TO_SKIP = 2
COLS_TO_SKIP = 3
MIN_GAP = 60
MAX_TAN = 3.3
FIRST_ROW_TO_SEARCH = 10

class tape_follow():
	def __init__(self):
		self.process_next = True
		self.next_tan = []
		self.previous_steering_angle = 0
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber('/camera/my_img/thresholded', Image, self.get_drive_instructions, queue_size = 1)
		self.pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 1)



	#Decides whether to process the message or not. Sends the drive instructions accordingly
	def get_drive_instructions(self, msg):
		tags_tan = []
		if self.process_next:
			tag_coordinates = self.find_tags(msg)
			if len(tag_coordinates[0]) > 1:
				tags_tan = self.convert_to_tan(tag_coordinates)
				if len(tags_tan) > 1:
					self.process_next = False
					self.next_tan.append(tags_tan[-1])
		else:
			self.process_next = True
			tags_tan.append(self.next_tan[0])
			self.next_tan = []

		if (len(tags_tan) > 0):
			self.drive(tags_tan)
	


	#Converts the coordinates array sent to it to the tan of the angles between
	#those coordinates. tag_coordinates should contain the array of x-coordinates
	#as the first element and y-coordinates as the second.
	def convert_to_tan(self, tag_coordinates):
		tags_tan = []
		
		tags_x_diff = np.ediff1d(tag_coordinates[0])
		tags_y_diff = np.ediff1d(tag_coordinates[1])
		tags_x_diff_float = []
		tags_x_diff = tags_x_diff + 0.0
		tags_tan = np.divide(tags_x_diff, tags_y_diff)
		
		return tags_tan



	#Finds the x and y coordinates of the tags and returns them as an array
	#which has the array of x and y coordinates as the first and second
	#element respectively
	def find_tags(self, msg):
		tags_x_avg, tags_y_avg = [], []
		tag_x, tag_y, row_count, rows_since_last_tag = 0, 0, 0, 0
		tag_found = False

		frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
		height, width = frame.shape[:2]
		#The first tag is at the center of the first line
		tags_x_avg.append(width/2)
		tags_y_avg.append(0)

		for i in xrange(FIRST_ROW_TO_SEARCH, height, ROWS_TO_SKIP):
			#Traverses row if at least one pixel is white.
			if sum(frame [height - i, :]) != 0:
				tag_found = True
				rows_since_last_tag = 0
				tag_y += i
				row_count += 1
				tag_x += self.find_avg_x(frame, width, height - i)
			else:
				#Finds and appends avg x & y coordinates of the tag
				if tag_found:
					tags_x_avg.append(tag_x / row_count)
					tags_y_avg.append(tag_y / row_count)
					row_count =  0
					tag_height = 0
					tag_x = 0 
					tag_y = 0
					tag_found = False
				rows_since_last_tag += 1
      
			#Exits if 2 tapes are found or there is more than a gap of MIN_GAP rows
			if len(tags_x_avg) == 3 or rows_since_last_tag * ROWS_TO_SKIP == MIN_GAP:
				break

		tag_coordinates = []
		tag_coordinates.append(tags_x_avg)
		tag_coordinates.append(tags_y_avg)
		return tag_coordinates



	#Given the frame, width and the row number, this will find and return the
	#average location of the white pixels in that line.
	def find_avg_x (self, frame, width, row):
		pixels_detected = 0
		pixel_location = 0
		for j in xrange(0, width - 1, COLS_TO_SKIP):
			if frame[row, j] > 0:
				pixels_detected += 1
				pixel_location += j
		
		if pixels_detected == 0:
			raise Exception("The number of pixels detected was 0")
		
		average_x = pixel_location / pixels_detected
		return average_x



	#Sends the drive instruction to the car based on the array of tan of angles
	#(between tags) passed to it in the parameter
	def drive(self, tags_tan):
		turning_factor = self.get_turning_factor(tags_tan)
		
		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()
		drive_msg.speed = MAX_SPEED - abs(turning_factor / 2)
		drive_msg.steering_angle = self.get_steering_angle(turning_factor)
		drive_msg.acceleration = 0.0
		drive_msg.jerk = 0.0
		drive_msg.steering_angle_velocity = 0.1
		drive_msg_stamped.drive = drive_msg
	
		self.previous_steering_angle = drive_msg.steering_angle
		self.pub.publish(drive_msg_stamped)


	#Calculates and returns the turning factor
	def get_turning_factor(self, tags_tan):
		turning_factor = tags_tan[0] / 4
		
		if turning_factor > 1 or any(tags_tan) > MAX_TAN:
			turning_factor = 1.0
		elif turning_factor < -1 or any(tags_tan) < -MAX_TAN:
			turning_factor = -1.0
			
		return turning_factor



	#Calculates the required steering angle based on the tan of the angles between
	#the tags. Also smoothens the steering.
	def get_steering_angle(self, turning_factor):
		steering_angle = -MAX_TURNING_ANGLE * turning_factor
		if (self.previous_steering_angle == -MAX_TURNING_ANGLE):
			if(steering_angle > 0):
				steering_angle = self.previous_steering_angle + steering_angle
		elif (self.previous_steering_angle == MAX_TURNING_ANGLE):
			if (steering_angle < 0):
				steering_angle = self.previous_steering_angle + steering_angle
        
		if self.process_next:
			steering_angle = (steering_angle + self.previous_steering_angle) / 2

		return steering_angle



if __name__== '__main__':
	rospy.init_node("tape_follow")
	tape_follow()
	rospy.spin()

