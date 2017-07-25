#!/usr/local/bin/python
import time
from std_msgs.msg import Bool
import sys
import rospkg

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import mission_control_utils

mission_control_utils.ros_init("test_custom_script_priority3")

test_counter3 = 10

for i in range(test_counter3, 0, -1):
    test_counter3 -= 1
    mission_control_utils.set_var("test_counter3", test_counter3)

    print "Countdown 6 in script priority 3: %d" % int(mission_control_utils.get_var("test_counter6", -1))
    print "Countdown priority 3: %d" % test_counter3

    time.sleep(1)
