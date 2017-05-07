#!/usr/bin/env python
import sys
import os
import rospy
import time
import rospkg
import unittest
from std_msgs.msg import String
from std_msgs.msg import Bool

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import behaviour
import mission_control_utils
from mission_control_utils_cache import Cache
from mission_control_utils_constants import Constants

rospy.init_node('test_mission_control_utils_set_var', anonymous=True)

class TestMissionControlUtilsSetVar(unittest.TestCase):

    start_asking = False

    stop_asking_pub = rospy.Publisher("/mission_control/test/mission_control_utils/stop_asking", Bool, queue_size=Constants.QUEUE_SIZE)

    def start_asking_callback(self, data):
        self.start_asking = True

    def test_set_variable_counter6(self):

        rospy.Subscriber("/mission_control/test/mission_control_utils/start_asking", Bool, self.start_asking_callback)
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.start_asking and time.time() < timeout_t:
            time.sleep(0.1)

        test_counter6_1 = mission_control_utils.get_var("test_counter6_1")

        self.stop_asking_pub.publish(True)

        self.assertTrue(test_counter6_1 == "15")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'test_mission_control_utils_set_var', TestMissionControlUtilsSetVar)
