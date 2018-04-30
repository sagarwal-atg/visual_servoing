#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from vision_works import Extract_center


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'
	        

def main():
    rospy.init_node('buoy_ramming_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Extract_center', Extract_center(), 
                               transitions={'hit':'BAR', 'in_pursuit':'Extract_center'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'done'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
