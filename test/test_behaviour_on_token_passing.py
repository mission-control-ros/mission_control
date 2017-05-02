#!/usr/bin/env python
import sys
import os
import rospy
import time
import rospkg
import unittest

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

from mission_control_utils_cache import Cache
from std_msgs.msg import Int32
from std_msgs.msg import Bool

class TestBehaviourTokenPassing(unittest.TestCase):

    token_got_prio = []

    prio6_paused = False

    prio6_resumed = False

    def token_callback(self, data):
        if len(self.token_got_prio) == 0:
            self.token_got_prio.append(data.data)
        else:
            last = self.token_got_prio[len(self.token_got_prio) - 1]
            if last != data.data:
                self.token_got_prio.append(data.data)
        print self.token_got_prio

    def prio6_pause_callback(self, data):
        self.prio6_paused = data.data

    def prio6_resume_callback(self, data):
        self.prio6_resumed = data.data

    def test_token_passing(self):
        rospy.init_node('test_behaviour_on_token_passing', anonymous=True)
        rospy.Subscriber("/mission_control/test/token_passing/has_token", Int32, self.token_callback)
        rospy.Subscriber("/mission_control/test/token_passing/priority6_paused", Bool, self.prio6_pause_callback)
        rospy.Subscriber("/mission_control/test/token_passing/priority6_resumed", Bool, self.prio6_resume_callback)

        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and len(self.token_got_prio) != 3 and time.time() < timeout_t:
            time.sleep(0.1)
        self.assertTrue(self.token_got_prio == [6, 3, 6])

        self.assertTrue(self.prio6_paused)
        self.assertTrue(self.prio6_resumed)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'test_behaviour_token_passing', TestBehaviourTokenPassing)
