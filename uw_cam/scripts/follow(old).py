#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import cv2
import numpy as np
from threading import Thread
from cv_bridge import CvBridge, CvBridgeError

LINES_TO_CHECK = 20
MAX_TURNING_ANGLE = 0.34
MAX_SPEED = 1.0
ROWS_TO_SKIP = 2
COLS_TO_SKIP = 3
MIN_GAP = 60

class tape_follow():
        def __init__(self):
		self.process_next = True
		self.next_tan = []
		self.previous_steering_angle = 0
		self.bridge = CvBridge()
                self.sub = rospy.Subscriber('/camera/my_img/thresholded', Image, self.tag_angles, queue_size=1)
                self.pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 1)
		
	#Finds the tan of the angles between the tags that it detects. 
	#The first tape is assumed to be right in front of it at the 
	#center of the first row of the image.
        def tag_angles(self, msg):
		if self.process_next:
			tags_x_avg = []
			tags_y_avg = []
			tags_x = []
			tag_height = 0
	                row_count = 0
			rows_since_last_tag =0
			tag_found = False
	                frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        	        height, width = frame.shape[:2]
			#The first tag is at the center of the first line
			tags_x_avg.append(width/2)
			tags_y_avg.append(0)
			#This should save the x and y coordinates of the first 2 tags 
			#in tags_x[] and tags_y[].
			#Starts searching for the tape from the second last row.
        	        for i in xrange(10, height, ROWS_TO_SKIP):
				pixels_detected = 0
				pixel_location = 0
				#only traverses the row if any of its pixels is not black. 
				if sum(frame [height - i, :]) != 0:
					tag_found = True
					rows_since_last_tag = 0
					tag_height += i
					row_count += 1	
        	                        for j in xrange(0, width - 1, COLS_TO_SKIP):
                	                       	if frame[height - i, j] > 0:
                        	                        pixels_detected += 1
                                	                pixel_location += j
					#appends the average of the white pixels in that row.
					tags_x.append(pixel_location / pixels_detected)
				else:
					#If tag was found in the previous rows, it finds its average x and y location.
					if tag_found:
						#append avg x
						tags_x_avg.append(sum(tags_x[:]) / row_count)
						#append avg y
						tags_y_avg.append(tag_height / row_count)
						#reset the parameters used to find the tag
						row_count = 0
						tags_x = []
						tag_height = 0
						tag_found = False
					rows_since_last_tag += 1
				#Exits from the loop in case 2 tapes are found or there is more than a gap
				#of MIN_GAP rows
				if len(tags_x_avg) == 3 or rows_since_last_tag * ROWS_TO_SKIP == MIN_GAP:
					break
			rospy.loginfo("The coordinates are")
			rospy.loginfo(tags_x_avg)
			rospy.loginfo(tags_y_avg)
			#Only drives if at least one tape has been found
			if len(tags_x_avg) > 1:
				tags_tan = []
				#Now tags_x_avg and tags_y_avg hold the change in x and y
				tags_x_avg = np.ediff1d(tags_x_avg)
				tags_y_avg = np.ediff1d(tags_y_avg)
				tags_x_avg_2 = []
				#converts the elements in tags_x_avg to floats and transfers them to tags_x_avg_2
				for i in xrange(0, len(tags_x_avg)):
					tags_x_avg_2.append(tags_x_avg[i] + 0.0)
				tags_tan = np.divide(tags_x_avg_2, tags_y_avg)
				rospy.loginfo(tags_x_avg)
				rospy.loginfo(tags_x_avg_2)
				if len(tags_tan) > 1:
					self.process_next = False
					self.next_tan.append(tags_tan[-1])
				rospy.loginfo("Tags tan is ")
				rospy.loginfo(tags_tan)
			  	self.drive(tags_tan)
		else:
			self.process_next = True
			tags_tan = []
			tags_tan.append(self.next_tan[0])
			rospy.loginfo("This is the predicted direction")
			rospy.loginfo(tags_tan)
			self.drive(tags_tan)
			self.next_tan = []
	
	#Sends the drive instruction to the car based on the array of tan of angles
	#passed to it in the parameter: tan_angles
        def drive(self, tag_angles):
		turning_factor = (tag_angles[0] / 4)
		#rospy.loginfo(turning_factor)
		if turning_factor > 1 or any(tag_angles) > 3.3: 
			turning_factor = 1.0
		elif turning_factor < -1 or any(tag_angles) < -3.3:
			turning_factor = -1.0
		steering_angle = -MAX_TURNING_ANGLE * turning_factor
		if (self.previous_steering_angle == -0.34):
			if (steering_angle > 0):
				steering_angle = self.previous_steering_angle + steering_angle
		elif (self.previous_steering_angle == 0.34):
			if (steering_angle < 0):
				steering_angle = self.previous_steering_angle + steering_angle
                drive_msg_stamped = AckermannDriveStamped()
                drive_msg = AckermannDrive()
                drive_msg.speed = MAX_SPEED - abs(turning_factor / 2)
		rospy.loginfo("The speed is ")
		rospy.loginfo(drive_msg.speed)
		if not self.process_next:
                	drive_msg.steering_angle = steering_angle
		else:
			drive_msg.steering_angle = (steering_angle + self.previous_steering_angle) / 2
		#saves the command it gives
		self.previous_steering_angle = steering_angle
                rospy.loginfo("The steering angle is")
                rospy.loginfo(drive_msg.steering_angle)
                drive_msg.acceleration = 0.0
                drive_msg.jerk = 0.0
                drive_msg.steering_angle_velocity = 0.1
                drive_msg_stamped.drive = drive_msg
                self.pub.publish(drive_msg_stamped)


if __name__== '__main__':
        rospy.init_node("tape_follow")
        tape_follow()
        rospy.spin()

