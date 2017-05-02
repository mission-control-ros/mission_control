#!/usr/bin/env python
import coverage_utils
coverage_utils.cov_start()

import rospy
import sys
import rospkg
rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)
import rospy
import behaviour_subprocess
from mission_control_utils_constants import Constants
from std_msgs.msg import Int32
from std_msgs.msg import Bool

def main():
    rospy.init_node('behaviour_subprocess', anonymous=True)
    beha = behaviour_subprocess.Behaviour_Subprocess()
    beha.set_debug_level(rospy.get_param('~debug', 0))
    beha.set_priority(rospy.get_param('~priority'))
    beha.set_active(rospy.get_param('~active'))
    beha.set_executable(rospy.get_param('~script'))

    """
    We need to make a little pause after start-up so some of the token release/request 
    do not go missing
    """
    rospy.sleep(float(rospy.get_param('~wait_before_startup', 1)))

    token_pub = rospy.Publisher("/mission_control/test/token_passing/has_token", Int32, queue_size=Constants.QUEUE_SIZE)
    prio6_paused_pub = rospy.Publisher("/mission_control/test/token_passing/priority6_paused", Bool, queue_size=Constants.QUEUE_SIZE)
    prio6_resumed_pub = rospy.Publisher("/mission_control/test/token_passing/priority6_resumed", Bool, queue_size=Constants.QUEUE_SIZE)

    prio6_paused = False
    print_report = False

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha.spin()
        rate.sleep()

        if beha._token and beha._running:
            token_pub.publish(beha._priority)

        if beha._priority == 6 and beha._paused:
            prio6_paused_pub.publish(True)
            prio6_paused = True

        if beha._priority == 6 and prio6_paused and not beha._paused:
            prio6_resumed_pub.publish(True)

        if beha._priority == 6:
            print_report = True

    coverage_utils.cov_stop(print_report)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
