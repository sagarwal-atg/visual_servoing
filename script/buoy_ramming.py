import roslib
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs.msg import *

import vision_works.buoy_hit

class bouy_ramming(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['hit'])

	def execute(self, userdata):
		rospy.loginfo