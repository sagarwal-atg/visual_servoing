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

import tensorflow as tf


class sub_env():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image, self.image_callback)
        self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", numpy_msg(geometry_msgs.Twist), queue_size=1)
        self.pose_sub = rospy.Subscriber("/rexrov/pose_gt",Odometry, self.pose_callback)
        self.position = np.array([0.,0.,0.])
        self.image = None
        self.bridge = CvBridge()
        self.createNetwork()

    def pose_callback(self, data):
        pos = data.pose.pose.position
        self.position[0] = pos.x
        self.position[1] = pos.y
        self.position[2] = pos.z
        print(self.position)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image = cv_image

    def weight_variable(self, shape):
        """ Initializa the weight variable."""
        initial = tf.truncated_normal(shape, stddev=0.01)
        return tf.Variable(initial)


    def bias_variable(self, shape):
        """ Initializa the bias variable."""
        initial = tf.constant(0.01, shape=shape)
        return tf.Variable(initial)


    def conv2d(self, x, W, stride):
        """ Define a convolutional layer."""
        return tf.nn.conv2d(x, W, strides=[1, stride, stride, 1], padding="SAME")


    def max_pool_2x2(self, x):
        """ Define a maxpooling layer."""
        return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding="SAME")


    def createNetwork(self):
        """ Create a convolutional network for estimating the Q value.
        Args:
        Returns:
            s: Input layer
            readout: Output layer with the Q-values for every possible action
        """
        # Initialize the network weights and biases.
        W_conv1 = self.weight_variable([8, 8, 4, 32])
        b_conv1 = self.bias_variable([32])

        W_conv2 = self.weight_variable([4, 4, 32, 64])
        b_conv2 = self.bias_variable([64])

        W_conv3 = self.weight_variable([3, 3, 64, 64])
        b_conv3 = self.bias_variable([64])

        W_fc1 = self.weight_variable([1600, 512])
        b_fc1 = self.bias_variable([512])

        W_fc2 = self.weight_variable([512, ACTIONS])
        b_fc2 = self.bias_variable([ACTIONS])

        # Input layer.
        s = tf.placeholder("float", [None, 80, 80, 4])

        # Hidden layers.
        h_conv1 = tf.nn.relu(self.conv2d(s, W_conv1, 4) + b_conv1)
        h_pool1 = self.max_pool_2x2(h_conv1)
        h_conv2 = tf.nn.relu(self.conv2d(h_pool1, W_conv2, 2) + b_conv2)
        h_conv3 = tf.nn.relu(self.conv2d(h_conv2, W_conv3, 1) + b_conv3)
        h_conv3_flat = tf.reshape(h_conv3, [-1, 1600])
        h_fc1 = tf.nn.relu(tf.matmul(h_conv3_flat, W_fc1) + b_fc1)

        # Output layer
        readout = tf.matmul(h_fc1, W_fc2) + b_fc2

        return s, readout

def main(args):
    ec = sub_env()
    rospy.init_node('sub_env', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
