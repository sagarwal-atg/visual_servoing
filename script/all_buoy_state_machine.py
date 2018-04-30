import roslib
import rospy
import smach
import smach_ros
from find_buoy import Find_Buoy
from tracking_hiting import Tracking_Hitting
import cv2

# define state Bar
class Stale(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'

def main():
    rospy.init_node('buoy_ramming_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])
    sm.userdata.sm_bbox = (0,0,0,0)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Find_Buoy', Find_Buoy(), 
                               transitions={'searching':'Find_Buoy', 'yellow':'Track_Hit', 'green':'Track_Hit', 'red':'Track_Hit'},
                               remapping={'bbox_in':'sm_bbox', 'bbox_out':'sm_bbox'})

        smach.StateMachine.add('Track_Hit', Tracking_Hitting(),
        						transitions={'hit':'Find_Buoy', 'in_pursuit':'Track_Hit'},
        						remapping={'tracker_bbox_in':'sm_bbox'})

        smach.StateMachine.add('Stale', Stale(), transitions={'outcome1':'done'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
