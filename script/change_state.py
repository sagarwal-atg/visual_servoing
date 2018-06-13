#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from rospy_message_converter import json_message_converter

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
    msg.model_name = data.name[1]
    data.pose[1].position.x = -10.0
    data.pose[1].position.y = -30.0
    data.pose[1].position.z = -45.0
    msg.pose = data.pose[1]
    msg.twist = data.twist[1]
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
