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
from mission_control_utils_constants import Constants
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class MoveToRoomOne(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['refresh','move_to_room_two'])
        
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
        rospy.loginfo('Executing MoveToRoomOne')
        if self.is_at_goal:
            self.goal_sent = False
            self.is_at_goal = False
            return 'move_to_room_two'
        elif not self.goal_sent or time.time() > self.next_send_time:

            rospy.loginfo('Setting goal in ONE')
            self.goal_sent = True
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(-5.303, 2.481, 0.000), Quaternion(0.000, 0.000, 1.000, -0.009))
           
            self.move_base.send_goal(goal)

            self.next_send_time = time.time() + 20

        else:
            self.is_at_goal = self.move_base.wait_for_result(rospy.Duration(2)) 

        return 'refresh'


class MoveToRoomTwo(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['refresh', 'move_to_room_one'])
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
        rospy.loginfo('Executing MoveToRoomTwo')
        if self.is_at_goal:
            self.goal_sent = False
            self.is_at_goal = False
            return 'move_to_room_one'
        elif not self.goal_sent or time.time() > self.next_send_time:

            rospy.loginfo('Setting goal in TWO')
            self.goal_sent = True
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(-17.055, -5.130, 0.000), Quaternion(0.000, 0.000, -0.717, 0.698))

            self.move_base.send_goal(goal)

            self.next_send_time = time.time() + 20

        else:
            self.is_at_goal = self.move_base.wait_for_result(rospy.Duration(0.5)) 

        return 'refresh'

sm = smach.StateMachine(outcomes=['end'])

with sm:
    smach.StateMachine.add('MoveToRoomOne', MoveToRoomOne(), 
                           transitions={'refresh':'MoveToRoomOne', 'move_to_room_two':'MoveToRoomTwo'})
    smach.StateMachine.add('MoveToRoomTwo', MoveToRoomTwo(), 
                           transitions={'refresh':'MoveToRoomTwo', 'move_to_room_one':'MoveToRoomOne'})
