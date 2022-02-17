import rospy
import smach
from smach_ros import SimpleActionState

from fsm_struct.msg import *

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed','failed'])
        self.counter = 0

    def execute(self, userdata):
        ret = 'succeed'

        return ret


class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'failed'])

    def execute(self, userdata):
        ret = 'succeed'

        return ret




# main
def main():
    rospy.init_node('gpm_qisda')

    # Create a SMACH state machine
    mainFsm = smach.StateMachine(outcomes=['end', 'failed'])

    # Open the container
    with mainFsm:
        # Add states to the container
        smach.StateMachine.add('INIT', Init(),
                               transitions={'succeed':'IDLE',
                                            'failed':'failed'})

        smach.StateMachine.add('IDLE',
                               SimpleActionState('idle', fsmAction),
                                  {'succeeded':'NAVIGATE',
                                   'aborted': 'FINISH',
                                   'preempted': 'failed'})

        smach.StateMachine.add('NAVIGATE',
                               SimpleActionState('navigate', fsmAction),
                                  {'succeeded':'TRACK_TARGET',
                                   'aborted': 'failed',
                                   'preempted': 'IDLE'})


        smach.StateMachine.add('TRACK_TARGET',
                               SimpleActionState('track_target', fsmAction),
                                  {'succeeded':'FINISH',
                                   'aborted': 'failed',
                                   'preempted': 'failed'})

        smach.StateMachine.add('ARRANGEMENT_PUT',
                               SimpleActionState('arrangement_put', fsmAction),
                                  {'succeeded':'IDLE',
                                   'aborted': 'failed',
                                   'preempted': 'failed'})

        smach.StateMachine.add('FINISH', Finish(),
                               transitions={'succeed':'end',
                                            'failed':'failed'})

    # Execute SMACH plan
    outcome = mainFsm.execute()

    if outcome == 'end':
        rospy.loginfo('mainFsm completed')
    elif outcome == 'failed':
        rospy.logerr('mainFsm failed')

if __name__ == '__main__':
    main()
