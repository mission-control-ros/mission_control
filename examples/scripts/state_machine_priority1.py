import sys
import rospy
import smach
import smach_ros
import time
import rospkg

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)
import mission_control_utils

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter1 = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO for state machine priority 1')
        if self.counter1 < 10:
            self.counter1 += 1
            mission_control_utils.set_var('counter1', self.counter1)
            time.sleep(1)
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.fuel_tank = True

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR for state machine priority 1')
        return 'outcome1'
        



sm = smach.StateMachine(outcomes=['outcome4'])

with sm:
    smach.StateMachine.add('FOO', Foo(), 
                           transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
    smach.StateMachine.add('BAR', Bar(), 
                           transitions={'outcome1':'FOO'})
