#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from rospy_message_converter import json_message_converter
import tf.transformations as tf_transform

'''
  orientation:
    x: -0.000220201445241
    y: 4.16686633275e-06
    z: 0.999021487799
    w: 0.0442268969121
'''

pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
test = False


def callback(data):
    # global test
    # print(data.pose[1].position.x + 10)
    # if not test:
    msg = ModelState()
    # msg.model_name = data.name[1]
    # data.pose[1].position.x = -10.0
    # data.pose[1].position.y = -30.0
    # data.pose[1].position.z = -45.0
    # msg.pose = data.pose[1]
    # msg.twist = data.twist[1]

    quaternion = tf_transform.quaternion_from_euler(0,0,135)

    msg.model_name = "rexrov"

    msg.pose.position.x = -10.0
    msg.pose.position.y = -30.0
    msg.pose.position.z = -45.0

    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    # msg.pose.orientation.x = -0.000220201445241 -0.00022618760879
    # msg.pose.orientation.y = 4.16686 4.41679362378e-06
    # msg.pose.orientation.z = 0.999021 0.999021446794
    # msg.pose.orientation.w = 0.0442268 0.0442277929149

    print(msg)
    pub.publish(msg)
    # test = True


def listener():
    rospy.init_node('change_model', anonymous=True)
    # print("hello")
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
