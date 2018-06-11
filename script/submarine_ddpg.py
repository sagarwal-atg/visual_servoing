#!/usr/bin/env python

import roslib
import rospy

import sys
import cv2
import numpy as np

from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import  Float32
from nav_msgs.msg import Odometry


class sub_env():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image, self.image_callback)
        self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", numpy_msg(geometry_msgs.Twist), queue_size=1)
        self.pose_sub = rospy.Subscriber("/rexrov/pose_gt",Odometry, self.pose_callback)

        self.position = np.array([0.,0.,0.])
        self.image = None
        self.linear_speed = 0.3
        self.bridge = CvBridge()
        self.reward = None
        self.terminal = None
        self.state = None

        self.state_dim = 100*75
        self.action_dim = 1
        self.action_bound = 1.0

        rospy.init_node('sub_env', anonymous=True)

    def step(self, yaw):
        msg = geometry_msgs.Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = yaw
        self.des_vel_pub.publish(msg)

    def pose_callback(self, data):
        pos = data.pose.pose.position
        self.position[0] = pos.x
        self.position[1] = pos.y
        self.position[2] = pos.z
        # print(self.position)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # self.image = cv2.resize(img, (0,0), fx=0.2, fy=0.2)
        self.image = cv2.resize(img, (100,75))
        # cv2.imshow("img", self.image )
        # print(self.image.flatten().shape)
        # print(self.image.shape)
        # cv2.waitKey(30)
        # print(self.process())
        self.state = self.image.flatten()

    def process(self):
        self.terminal = False
        self.state = self.image.flatten()
        self.reward = 0
        # checking y
        if self.position[1] < -40 or self.position[1] > -20:
            self.reward = -1
            self.terminal = True
        if self.position[0] < -33:
            self.reward = -2
            self.terminal = True
        if -31 < self.position[0] < -29 and -26 < self.position[1] < -24:
            self.reward = 5
            self.terminal = True

        return self.state, self.reward, self.terminal


def main(args):
    # ec = sub_env()
    rospy.init_node('sub_env', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
