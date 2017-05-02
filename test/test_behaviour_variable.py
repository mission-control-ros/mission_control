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
from std_msgs.msg import String

rospy.init_node('test_behaviour_on_token_passing', anonymous=True)

class TestBehaviourVariables(unittest.TestCase):

    test_counter6_foo = None
    test_counter6_foo_got = False

    test_counter6_bar = None
    test_counter6_bar_got = False

    test_counter3_foo = None
    test_counter3_foo_got = False

    test_default = None
    test_default_got = False


    def counter6_foo_callback(self, data):
        self.test_counter6_foo = data.data
        self.test_counter6_foo_got = True

    def counter6_bar_callback(self, data):
        self.test_counter6_bar = data.data
        self.test_counter6_bar_got = True

    def counter3_foo_callback(self, data):
        self.test_counter3_foo = data.data
        self.test_counter3_foo_got = True

    def default_callback(self, data):
        self.test_default = data.data
        self.test_default_got = True

    def test_variable_counter6_foo(self):

        rospy.Subscriber("/mission_control/test/variable/test_counter6_foo", String, self.counter6_foo_callback)
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.test_counter6_foo_got and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.test_counter6_foo == "10")

    def test_variable_counter6_bar(self):

        rospy.Subscriber("/mission_control/test/variable/test_counter6_bar", String, self.counter6_bar_callback)
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.test_counter6_bar_got and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.test_counter6_bar == "100")

    def test_variable_counter3_foo(self):

        rospy.Subscriber("/mission_control/test/variable/test_counter3_foo", String, self.counter3_foo_callback)
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.test_counter3_foo_got and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.test_counter3_foo == "40")

    def test_variable_default(self):

        rospy.Subscriber("/mission_control/test/variable/test_default", String, self.default_callback)
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.test_default_got and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.test_default == "777")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'test_behaviour_token_passing', TestBehaviourVariables)
