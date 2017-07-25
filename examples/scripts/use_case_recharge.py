import sys
import rospy
import smach
import smach_ros
import time
import rospkg
from std_msgs.msg import Bool

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)
import mission_control_utils
from mission_control_utils_constants import Constants
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class MoveToRecharge(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['refresh','recharge'])
        
        self.goal_sent = False
        self.is_at_goal = False
        self.next_send_time = time.time()

	rospy.on_shutdown(self.shutdown)
	
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	self.move_base.wait_for_server(rospy.Duration(5))

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

    def execute(self, userdata):
        rospy.loginfo('Executing MoveToRecharge')
        if self.is_at_goal:
            self.goal_sent = False
            self.is_at_goal = False
            return 'recharge'
        elif not self.goal_sent or time.time() > self.next_send_time:

            rospy.loginfo('Setting goal in MoveToRecharge')
            self.goal_sent = True
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(-10.230, -5.668, 0.000), Quaternion(0.000, 0.000, -0.697, 0.717))
           
            self.move_base.send_goal(goal)

            self.next_send_time = time.time() + 20

        else:
            self.is_at_goal = self.move_base.wait_for_result(rospy.Duration(2)) 

        return 'refresh'


class Recharge(smach.State):

    recharge_pub = rospy.Publisher("/mission_control/use_case/battery/recharge", Bool, queue_size=Constants.QUEUE_SIZE)

    def __init__(self):
        smach.State.__init__(self, outcomes=['refresh', 'end'])

        self.recharging = 0

    def execute(self, userdata):
        rospy.loginfo('Executing Recharge')

        battery_level = int(mission_control_utils.get_var('battery_level'))
        #rospy.loginfo('Battery level in recharge ' + str(battery_level))

        if battery_level > 95:
            mission_control_utils.set_var('recharging', 0, 2)
            return 'end'

        mission_control_utils.set_var('recharging', 1, 2)

        self.recharge_pub.publish(True)

        time.sleep(0.5)

        return 'refresh'

sm = smach.StateMachine(outcomes=['end'])

with sm:
    smach.StateMachine.add('MoveToRecharge', MoveToRecharge(), 
                           transitions={'refresh':'MoveToRecharge', 'recharge':'Recharge'})
    smach.StateMachine.add('Recharge', Recharge(), 
                           transitions={'refresh':'Recharge', 'end':'end'})

if __name__ == '__main__':
    mission_control_utils.ros_init('use_case_recharge')
    sm.execute()
