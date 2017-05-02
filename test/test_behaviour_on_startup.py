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

import behaviour
from mission_control_utils_cache import Cache
from std_msgs.msg import Int32

class TestBehaviourTokenOnStartup(unittest.TestCase):

    msg_got = False
    token_got_prio = 0

    def setUp(self):
        self.msg_got = False
        self.token_got_prio = 0

    def tearDown(self):
        self.msg_got = False
        self.token_got_prio = 0

    def callback(self, data):
        self.token_got_prio = data.data
        self.msg_got = True

    def test_token_on_startup(self):
        rospy.init_node('test_behaviour_on_startup', anonymous=True)
        rospy.Subscriber("/mission_control/test/startup/has_token", Int32, self.callback)
        timeout_t = time.time() + 10.0
        while not rospy.is_shutdown() and not self.msg_got and time.time() < timeout_t:
            time.sleep(0.1)
        self.assertTrue(self.token_got_prio == 1)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'test_behaviour_on_startup', TestBehaviourTokenOnStartup)
