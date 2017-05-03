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

from mission_control_utils_constants import Constants
import mission_control_utils
from std_msgs.msg import Bool
from std_msgs.msg import Int32

rospy.init_node('test_watchdog_fail_safe', anonymous=True)

class TestWatchdogFailSafe(unittest.TestCase):

    fail_safe_started = False
    fail_safe_finished = False
    test_counter6_foo = None
    test_counter6_bar = None

    def fail_safe_started_callback(self, data):
        self.fail_safe_started = data.data

    def fail_safe_finished_callback(self, data):
        self.fail_safe_finished = data.data

    def fail_safe_foo_callback(self, data):
        self.test_counter6_foo = data.data

    def fail_safe_bar_callback(self, data):
        self.test_counter6_bar = data.data

    def test_fail_safe_run_and_variables(self):
        rospy.Subscriber("/mission_control/test/fail_safe/started", Bool, self.fail_safe_started_callback)
        rospy.Subscriber("/mission_control/test/fail_safe/finished", Bool, self.fail_safe_finished_callback)
        rospy.Subscriber("/mission_control/test/fail_safe/test_counter6_foo", Int32, self.fail_safe_foo_callback)
        rospy.Subscriber("/mission_control/test/fail_safe/test_counter6_bar", Int32, self.fail_safe_bar_callback)

        bar = None
        foo = None

        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.fail_safe_finished and time.time() < timeout_t:
            if self.fail_safe_started:
                foo = int(mission_control_utils.get_var("test_counter6_foo", -1)) #Getting data from fail safe node
                bar = int(mission_control_utils.get_var("test_counter6_bar", -1)) #Getting data from fail safe node
            time.sleep(0.1)

        self.assertTrue(self.fail_safe_finished)
        self.assertTrue(self.test_counter6_foo == 10)
        self.assertTrue(self.test_counter6_bar == 100)
        self.assertTrue(foo == 10)
        self.assertTrue(bar == 100)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'test_watchdog_fail_safe', TestWatchdogFailSafe)
