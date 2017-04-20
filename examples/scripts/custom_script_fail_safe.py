#!/usr/local/bin/python
import rospkg
import sys
import time

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import mission_control_utils

mission_control_utils.ros_init("custom_script_fail_safe")

final = 3

while final != 0:

    print "Countdown in fail safe: %d" % final
    print "Counter 6 in fail safe script: %d" % int(mission_control_utils.get_var("counter6", -1))

    time.sleep(1)

    final -= 1
