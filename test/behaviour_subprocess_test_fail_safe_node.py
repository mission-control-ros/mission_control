#!/usr/bin/env python
import coverage_utils
coverage_utils.cov_start()

import rospy
import rospkg
import sys
rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import behaviour_subprocess_fail_safe
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from mission_control_utils_constants import Constants

def main():
    rospy.init_node('behaviour_subprocess_fail_safe', anonymous=True)
    beha_fail_safe = behaviour_subprocess_fail_safe.Behaviour_Subprocess_Fail_Safe()
    beha_fail_safe.set_debug_level(rospy.get_param('~debug', 0))
    beha_fail_safe.set_executable(rospy.get_param('~script'))

    """
    We need to make a little pause after start-up so some of the token release/request 
    do not go missing
    """
    rospy.sleep(float(rospy.get_param('~wait_before_startup', 1)))

    started_pub = rospy.Publisher("/mission_control/test/fail_safe/started", Bool, queue_size=Constants.QUEUE_SIZE)
    finished_pub = rospy.Publisher("/mission_control/test/fail_safe/finished", Bool, queue_size=Constants.QUEUE_SIZE)
    test_counter6_foo_pub = rospy.Publisher("/mission_control/test/fail_safe/test_counter6_foo", Int32, queue_size=Constants.QUEUE_SIZE)
    test_counter6_bar_pub = rospy.Publisher("/mission_control/test/fail_safe/test_counter6_bar", Int32, queue_size=Constants.QUEUE_SIZE)

    ran = False

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha_fail_safe.spin()
        rate.sleep()

        if beha_fail_safe._active:
            test_counter6_foo_pub.publish(int(beha_fail_safe.get_var("test_counter6_foo", -1)))
            test_counter6_bar_pub.publish(int(beha_fail_safe.get_var("test_counter6_bar", -1)))

        if beha_fail_safe._running and not ran:
            started_pub.publish(True)
            ran = True

        if not beha_fail_safe._running and ran:
            finished_pub.publish(True)

    coverage_utils.cov_stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
