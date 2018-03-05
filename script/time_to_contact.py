#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import CMT
import numpy as np
import util

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
import geometry_msgs.msg as geometry_msgs
from cv_bridge import CvBridge, CvBridgeError

class time_to_contact:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image,self.callback)
        self.prev_h = 0
        self.bridge = CvBridge()

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([50,50,60])
        upper_orange = np.array([255,255,90])


        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        res = cv2.bitwise_and(frame,frame, mask= mask)
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        # img_filt = cv2.medianBlur(data, 5)
        # data = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # # ret, img_th = cv2.threshold(img_filt, 90, 255, cv2.THRESH_BINARY)
        # img_th = cv2.adaptiveThreshold(img_filt,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,4)
        # cv2.imshow("asdasd", img_th)
        img_th = res
        # im2, contours, hierarchy = cv2.findContours(img_th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(frame, contours, -1, (0,255,0), 3)

        # if len(contours) != 0:
        #     # draw in blue the contours that were founded
        #     cv2.drawContours(frame, contours, -1, 255, 3)
        #
        #     #find the biggest area
        #     c = max(contours, key = cv2.contourArea)
        #
        #     x,y,w,h = cv2.boundingRect(c)
        #     # draw the book contour (in green)
        #     cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        #
        return frame

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        # cv2.imshow("Final", cv_image)
        cv_image = self.detect(cv_image)
        cv2.imshow("Final", cv_image)
        cv2.waitKey(100)

def main(args):
  ec = time_to_contact()
  rospy.init_node('time_to_contact', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
