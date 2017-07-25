#!/usr/bin/env python
import coverage_utils
coverage_utils.cov_start()

import rospkg
import sys
rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)
import rospy
import behaviour
from mission_control_utils_constants import Constants
from std_msgs.msg import Int32

def main():
    rospy.init_node('behaviour', anonymous=True)
    beha = behaviour.Behaviour()
    beha.set_debug_level(rospy.get_param('~debug', 0))
    beha.set_priority(rospy.get_param('~priority'))
    beha.set_active(rospy.get_param('~active'))
    beha.set_executable(rospy.get_param('~script'))

    """
    We need to make a little pause after start-up so some of the token release/request 
    do not go missing
    """
    rospy.sleep(float(rospy.get_param('~wait_before_startup', 1)))

    token_pub = rospy.Publisher("/mission_control/test/startup/has_token", Int32, queue_size=Constants.QUEUE_SIZE)

    print_report = False
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha.spin()
        rate.sleep()
        if beha._token:
            token_pub.publish(beha._priority)
            print_report = True

    coverage_utils.cov_stop(print_report)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
