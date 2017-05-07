#!/usr/bin/env python
import sys
import os
import rospy
import time
import rospkg
import unittest
from std_msgs.msg import Int32
from std_msgs.msg import Bool

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import behaviour
import mission_control_utils
from mission_control_utils_cache import Cache
from mission_control_utils_constants import Constants

rospy.init_node('test_mission_control_utils_get_var', anonymous=True)

class TestMissionControlUtilsGetVar(unittest.TestCase):

    start_setting = False

    answer_got = False
    answer_counter6 = None

    expired_answer_got = False
    expired_answer_counter6 = None

    def start_setting_callback(self, data):
        self.start_setting = True

    def answer_test_counter6(self, data):
        self.answer_counter6 = data.data
        self.answer_got = True

    def expired_answer_test_counter6(self, data):
        self.expired_answer_counter6 = data.data
        self.expired_answer_got = True

    def test_get_variable_counter6(self):

        rospy.Subscriber("/mission_control/test/mission_control_utils/start_setting", Bool, self.start_setting_callback)
        rospy.Subscriber("/mission_control/test/mission_control_utils/answer_test_counter6", Int32, self.answer_test_counter6)
        rospy.Subscriber("/mission_control/test/mission_control_utils/expired_answer_test_counter6", Int32, self.expired_answer_test_counter6)

        test_counter6 = 456

        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.start_setting and time.time() < timeout_t:
            time.sleep(0.1)

        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.answer_got and time.time() < timeout_t:
            mission_control_utils.set_var("test_counter6", test_counter6, 5)
            time.sleep(0.5)

        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.expired_answer_got and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.answer_counter6 == test_counter6)
        self.assertTrue(self.expired_answer_counter6 == -1)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'test_mission_control_utils_get_var', TestMissionControlUtilsGetVar)
