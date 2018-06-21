#!/usr/bin/env python

import roslib
import rospy

import sys
import cv2
import numpy as np
import time

import tf.transformations as tf_transform


from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import  Float32
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from std_msgs.msg import  Float32


class test():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image, self.image_callback)
        self.s_t = None
        self.first = True
        self.bridge = CvBridge()

    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.first:
            x_t = cv2.cvtColor(cv2.resize(cv_image, (80, 80)), cv2.COLOR_BGR2GRAY)
            ret, x_t = cv2.threshold(x_t, 1, 255, cv2.THRESH_BINARY)
            self.s_t = np.stack((x_t, x_t, x_t, x_t), axis=2)
            self.first = False
        else:
            x_t = cv2.cvtColor(cv2.resize(cv_image, (80, 80)), cv2.COLOR_BGR2GRAY)
            # ret, x_t = cv2.threshold(x_t, 1, 255, cv2.THRESH_BINARY)
            self.s_t = np.stack((x_t, self.s_t[:,:,0], self.s_t[:,:,1], self.s_t[:,:,2]), axis=2)

        cv2.imshow("image", self.s_t)
        cv2.waitKey(30)
        self.state = self.s_t.flatten()
        print(self.state.shape)

def main(args):
  ec = test()
  rospy.init_node('vision_test', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
