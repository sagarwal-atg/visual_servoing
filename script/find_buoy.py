#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time

import sys

class Find_Buoy(smach.State):
	"""docstring for Find_Buoy"""
	def __init__(self):
		smach.State.__init__(self,
							 outcomes=['searching','yellow','red','green'],
							 input_keys=['bbox_in'],
							 output_keys=['bbox_out'])

		self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image,self.callback)
		self.bridge = CvBridge()
		self.h = 0
		self.w = 0
		self.center = None
		self.init_img = None
		self.yellow = 0
		self.green = 1
		self.red = 2

		self.green_bbox = None
		self.yellow_bbox = None
		self.red_bbox = None

		self.yellow_done = False
		self.red_done = False
		self.green_done = False

		self.state = 0
		self.curr_state = 0
		self.once = True

		self.this_state_exec = False

	def execute(self, userdata):
		# @TODO check if self.state can be just checked for even
		print("state", self.state)
		if self.state == 0 or self.state == 2 or self.state == 4 :
			self.this_state_exec = True
			return 'searching'
		elif self.state == 1:
			userdata.bbox_out = self.yellow_bbox
			# self.yellow_done = True
			self.this_state_exec = False
			self.state += 1
			return 'yellow'
		elif self.state == 3:
			userdata.bbox_out = self.green_bbox
			# self.green_done = True
			self.this_state_exec = False
			self.state += 1
			return 'green'
		elif self.state == 5:
			userdata.bbox_out = self.red_bbox
			# self.red_done = True
			self.this_state_exec = False
			return 'red'

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.init_img = cv_image
		# if self.state == 0 and not self.yellow_done:
		# 	self.extract_init_img(self.init_img)
		# elif self.state == 2 and not self.red_done:
		# 	self.extract_init_img(self.init_img)
		# elif self.state == 4 and not self.green_done:
		# 	self.extract_init_img(self.init_img)
		cv2.imshow("Final", cv_image)
		if self.this_state_exec:
			self.extract_init_img(self.init_img)
		
		cv2.waitKey(10)

	def extract_init_img(self, cv_frame):
		buoy = cv2.imread('buoy_sim.png')
		templ_gray = cv2.cvtColor(buoy, cv2.COLOR_BGR2GRAY)
		frame_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
		self.h , self.w = frame_gray.shape
		all_tl, all_br = self.tem_match(cv_frame, frame_gray, templ_gray)
		self.green_bbox = self.get_bbox(all_tl[self.green], all_br[self.green])
		self.yellow_bbox = self.get_bbox(all_tl[self.yellow], all_br[self.yellow])
		self.red_bbox = self.get_bbox(all_tl[self.red], all_br[self.red])

		if len(all_tl) == 3:
			self.state += 1
			self.once = False

	def get_bbox(self,tl,br):
		bbox = (tl[0], tl[1], br[0]-tl[0], br[1]-tl[1])
		return bbox

	def tem_match(self, orig, src, templ):
		img = src
		img2 = img.copy()
		template = templ
		w, h = template.shape[::-1]

		all_tl = [0,0,0]
		all_br = [0,0,0]
		all_point = [0,0,0]

		tl = bl = None
		point = None

		methods = ['cv2.TM_CCOEFF_NORMED']
		for meth in methods:
			img = img2.copy()
			resize_i = img2.copy()
			method = eval(meth)
			orig_res = None
			for i in range(1):
				resize_i = cv2.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv2.INTER_AREA)
				# Apply template Matching
				res = cv2.matchTemplate(resize_i, template, method)
				if i == 0:
				    orig_res = res
				threshold = 0.70
				loc = np.where( res >= threshold)
				first_point = 0
				for pt in zip(*loc[::-1]):
					point = pt
					# green
					if first_point == 0:
						all_point[self.green] = pt
						cv2.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,255,0), 1)
						first_point += 1

					elif all_point[self.green][0] > (pt[0] + 50) and first_point == 1:
						all_point[self.yellow] = pt
						cv2.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,255,255), 1)
						first_point += 1

					elif all_point[self.green][0] < (pt[0] + 50) and first_point == 2:
						all_point[self.red] = pt
						cv2.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,0,255), 1)
						first_point += 1

		#yellow
		all_tl[self.yellow] =  (all_point[self.yellow][0],all_point[self.yellow][1])
		all_br[self.yellow] = ((all_point[self.yellow][0] + w),(all_point[self.yellow][1] + h))
		# green
		all_tl[self.green] = ( (all_point[self.green][0],all_point[self.green][1]))
		all_br[self.green] = ( (all_point[self.green][0] + w),(all_point[self.green][1] + h))
		# red
		all_tl[self.red] = ( (all_point[self.red][0],all_point[self.red][1]))
		all_br[self.red] = ( (all_point[self.red][0] + w),(all_point[self.red][1] + h))

		# self.center = center = ((br[0]-tl[0])/2 , (br[1]-tl[1])/2)
		# cv2.circle(orig,(tl[0] + center[0], tl[1] + center[1]), 1, (0,0,255), 2)
		cv2.imshow('Matching Result', orig_res)
		cv2.imshow('Detected Point', orig)
		cv2.waitKey(10)
		return all_tl, all_br


def main(args):
  ec = Find_Buoy()
  rospy.init_node('target_follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
