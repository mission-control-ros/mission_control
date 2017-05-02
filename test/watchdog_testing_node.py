#!/usr/bin/env python
import coverage_utils
coverage_utils.cov_start()

import rospy
import rospkg
import sys
rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import watchdog
from mission_control_utils_constants import Constants
from std_msgs.msg import Bool


def main():
    rospy.init_node('watchdog', anonymous=True)

    watch = watchdog.Watchdog(rospy.get_param('~node_dead_after', 2), rospy.get_param('~debug_file', ''))
    watch.set_debug(rospy.get_param('~debug', False))

    node_not_pub = rospy.Publisher("/mission_control/test/watchdog/node_not_ok", Bool, queue_size=Constants.QUEUE_SIZE)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        watch.check_nodes()
        rate.sleep()
        node_not_pub.publish(watch._all_ok)

    coverage_utils.cov_stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
