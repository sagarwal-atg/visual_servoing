#!/usr/bin/env python

import sys
import cv2
import numpy as np

import roslib
import rospy
import smach
import smach_ros

from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import  Float32

import time

class Tracking_Hitting(smach.State):
	"""docstring for Tracking_Hitting"""
	def __init__(self):

		self.tracker_yellow = cv2.TrackerKCF_create()
		self.tracker_green = cv2.TrackerKCF_create()
		self.tracker_red = cv2.TrackerKCF_create()

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image,self.callback)
		self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", numpy_msg(geometry_msgs.Twist), queue_size=1)
		self.jerk_sub = rospy.Subscriber("/jerk",Float32,self.jerk_callback)

		smach.State.__init__(self,
							 outcomes=['hit', 'in_pursuit'],
							 input_keys=['tracker_bbox_in'])

		self.first = True
		self.init_bbox = None
		self.init_img = None
		self.h = None
		self.w = None
		self.count = 0
		self.got_bbox = False

		self.hit_buoy = None

		self.hit_buoy_yellow = False
		self.hit_buoy_green = False
		self.hit_buoy_red = False

		self.center = None
		self.bbox_h = None
		self.linear_speed_x = 0.40
		self.k_yaw = 0.0005
		self.k_alt = 0.0010

		self.buoy_counter = 0


	def execute(self, userdata):

		if self.hit_buoy_yellow:
			print("yellow")
			self.first = True
			self.hit_buoy_yellow = False
			self.got_bbox = False
			return 'hit'
		if self.hit_buoy_green:
			print("green")
			self.first = True
			self.hit_buoy_green = False
			self.got_bbox = False
			return 'hit'
		if self.hit_buoy_red:
			print("red")
			self.first = True
			self.hit_buoy_red = False
			self.got_bbox = False
			return 'hit'
		else:
			print("*************************************")
			print(self.hit_buoy)
			print(self.buoy_counter)
			print(self.hit_buoy_red)
			print(self.hit_buoy_yellow)
			print(self.hit_buoy_green)
			print(self.first)
			print(userdata.tracker_bbox_in)
			print("*************************************")
			self.hit_buoy = False
			self.init_bbox = userdata.tracker_bbox_in
			self.got_bbox = True
			return 'in_pursuit'

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		if self.first and self.got_bbox:
			self.init_img = cv_image
			self.first = False
			self.h = cv_image.shape[0]
			self.w = cv_image.shape[1]
			bbox = self.init_bbox

			if self.buoy_counter == 0:
				bbox = self.init_bbox
				self.tracker_yellow.init(self.init_img, bbox)
			elif self.buoy_counter == 1:
				bbox = self.init_bbox
				self.tracker_green.init(self.init_img, bbox)
				print("**********************************")
				print(bbox)
				print("**********************************")
			elif self.buoy_counter == 2:
				bbox = self.init_bbox
				self.tracker_red.init(self.init_img, bbox)

			self.count += 1

		elif self.count and self.got_bbox:
			self.count += 1
			# print("Tracking " + str(self.count))
			self.opencv_tracker(cv_image)
			if not self.hit_buoy:
				self.target_follower("forward")
		elif self.got_bbox:
			self.count += 1
		# cv2.imshow("Final", cv_image)
		cv2.waitKey(10)

	def target_follower(self, direc):
		msg = geometry_msgs.Twist()
		if direc == 'forward':
			print("Front")
			d_alt = self.k_alt*(self.h/2 - self.center[1])
			d_yaw = self.k_yaw*(self.w/2 - self.center[0])

			msg.linear.x = self.linear_speed_x
			msg.linear.z = d_alt
			msg.angular.z = d_yaw

		elif direc == 'backward':
			msg.linear.x = -self.linear_speed_x
			msg.linear.z = 0
			msg.angular.z = 0

		self.des_vel_pub.publish(msg)

	def opencv_tracker(self,img):
		frame = img.copy()
		ok = None
		bbox = None
		if self.buoy_counter == 0:
			ok, bbox = self.tracker_yellow.update(frame)
		if self.buoy_counter == 1:
			ok, bbox = self.tracker_green.update(frame)
		if self.buoy_counter == 2:
			ok, bbox = self.tracker_red.update(frame)

		# Draw bounding box
		if ok:
			# Tracking success
			tl = (int(bbox[0]), int(bbox[1]))
			br = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
			cv2.rectangle(frame, tl, br, (255,0,0), 2, 1)
			self.bbox_h = bbox[2]/2
			self.center = center = ( tl[0] + (br[0]-tl[0])/2 , tl[1] + (br[1]-tl[1])/2)
		else :
			print("Tracking failure")
			cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
		cv2.imshow('tracker',frame)

	def jerk_callback(self, msg):
		jerk = msg.data
		if jerk > 1.5:
			print("Backward")
			if self.buoy_counter == 0:
				self.hit_buoy_yellow = True
				self.hit_buoy = True
				self.buoy_counter += 1

			elif self.buoy_counter == 1:
				self.hit_buoy_green = True
				self.hit_buoy = True
				self.buoy_counter += 1

			elif self.buoy_counter == 2:
				self.hit_buoy_red = True
				self.hit_buoy = True
				self.buoy_counter += 1

			self.target_follower("backward")
