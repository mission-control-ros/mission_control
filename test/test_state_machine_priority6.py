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
        self.test_counter6 = 10

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO for priority 6')
        if self.test_counter6 != 0:
            self.test_counter6 -= 1
            rospy.loginfo("Countdown for state machine priority 6: %d", self.test_counter6)
            mission_control_utils.set_var('test_counter6', self.test_counter6, 5)
            time.sleep(1)
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR for priority 6')
        return 'outcome1'
        




sm = smach.StateMachine(outcomes=['outcome4'])

with sm:
    smach.StateMachine.add('FOO', Foo(), 
                           transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
    smach.StateMachine.add('BAR', Bar(), 
                           transitions={'outcome1':'FOO'})

if __name__ == '__main__':
    mission_control_utils.ros_init('test_state_machine_priority6')
    sm.execute()
