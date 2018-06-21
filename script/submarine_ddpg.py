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


class sub_env():
    def __init__(self):

        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image, self.image_callback)
        self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", numpy_msg(geometry_msgs.Twist), queue_size=1)
        self.pose_sub = rospy.Subscriber("/rexrov/pose_gt",Odometry, self.pose_callback)
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.jerk_sub = rospy.Subscriber("/jerk",Float32,self.jerk_callback)

        self.position = np.array([0.,0.,0.])
        self.image = None
        self.linear_speed = 0.3
        self.bridge = CvBridge()
        self.reward = None
        self.terminal = None
        self.state = None

        self.hit = False

        self.state_dim = 80*80
        self.action_dim = 2
        self.action_bound = 0.5

        self.now = time.time()

        rospy.init_node('sub_env', anonymous=True)
        # rospy.spin()

    def jerk_callback(self, msg):
		jerk = msg.data
		if jerk > 1.5:
			self.hit = True

    def timeout(self):
        return time.time() > self.now + 150

    def step(self, yaw, depth):
        # print(yaw)
        msg = geometry_msgs.Twist()
        msg.linear.x = self.linear_speed

        msg.angular.z = yaw
        msg.linear.z = depth

        self.des_vel_pub.publish(msg)

    def pose_callback(self, data):
        pos = data.pose.pose.position
        self.position[0] = pos.x
        self.position[1] = pos.y
        self.position[2] = pos.z
        # print(self.position)

    def starting_region_check(self):
        if -9.5 > self.position[0] > -10.5 and -29.5 > self.position[1] > -30.5:
            return True

    def image_callback(self, data):
        # print("Recieved Image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # self.image = cv2.resize(img, (0,0), fx=0.2, fy=0.2)
        self.image = cv2.resize(img, (80,80))
        # cv2.imshow("img", self.image )
        # print(self.image.flatten().shape)
        # print(self.image.shape)
        # cv2.waitKey(30)
        # print(self.process())
        self.state = self.image.flatten()

    def reset(self):
        msg = ModelState()
        quaternion = tf_transform.quaternion_from_euler(0,0,135)

        msg.model_name = "rexrov"

        msg.pose.position.x = -10.0
        msg.pose.position.y = -30.0
        msg.pose.position.z = -45.0

        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        self.hit = False

        self.model_state_pub.publish(msg)

        time.sleep(1)

        self.curr_time = time.time() - self.now

        self.now = time.time()

    def process(self):
        self.terminal = False
        self.state = self.image.flatten()
        self.reward = 0

        # checking y
        if self.position[1] < -35 or self.position[1] > -15:
            self.reward = -1
            self.terminal = True
            print("Y dimension exceeded")

        # checking z
        if self.position[2] < -48 or self.position[2] > -35:
            self.reward = -1
            self.terminal = True
            print("Z dimension exceeded")

        # checking x
        if self.position[0] < -28 or self.position[0] > -9:
            self.reward = -1
            self.terminal = True
            print("X dimension exceeded")

        if self.timeout():
            self.reward = -1
            self.terminal = True
            print("Timeout " + str(time.time() - self.now))

        # checking buoy
        if -26 < self.position[0] < -24 and -26 < self.position[1] < -24:
            self.reward = 5
            print("Dice Hit")
            self.terminal = True

        time_now = time.time() - self.now

        if self.hit and time_now > 30.0:
            self.reward = 5
            print("Buoy Hit: Jerk")
            print(self.position)
            self.terminal = True

        # print(self.state, self.reward, self.terminal)
        # print(self.position)

        return self.state, self.reward, self.terminal
