#!/usr/bin/env python
import coverage_utils
coverage_utils.cov_start()

import rospy
import rospkg
import sys
rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

from mission_control_utils_constants import Constants


def main():
    coverage_utils.cov_stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
