#!/usr/local/bin/python
import time
from std_msgs.msg import Bool
import signal
import sys
import rospkg

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import mission_control_utils

mission_control_utils.ros_init("custom_script_priority6")

test_counter6 = 10
execute = False

for i in range(test_counter6, 0, -1):
    test_counter6 -= 1
    mission_control_utils.set_var("test_counter6", test_counter6)

    print "Countdown priority 6: %d" % test_counter6

    time.sleep(1)
