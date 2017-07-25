#!/usr/bin/env python
import coverage_utils
coverage_utils.cov_start()

import sys
import os
import rospy
import rospkg
import unittest

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

from mission_control_utils_cache import Cache

class TestFake(unittest.TestCase):

    def test_fake(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'show_test_coverage_fake_test', TestFake)
    coverage_utils.cov_stop()
