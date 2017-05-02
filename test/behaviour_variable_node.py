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
from std_msgs.msg import String

def main():
    rospy.init_node('behaviour_variable', anonymous=True)
    beha = behaviour.Behaviour()
    beha.set_debug_level(rospy.get_param('~debug', 0))
    beha.set_priority(rospy.get_param('~priority'))
    beha.set_active(rospy.get_param('~active'))
    beha.set_executable(rospy.get_param('~state_machine'))

    """
    We need to make a little pause after start-up so some of the token release/request 
    do not go missing
    """
    rospy.sleep(float(rospy.get_param('~wait_before_startup', 1)))

    test_counter6_foo_pub = rospy.Publisher("/mission_control/test/variable/test_counter6_foo", String, queue_size=Constants.QUEUE_SIZE)
    test_counter6_bar_pub = rospy.Publisher("/mission_control/test/variable/test_counter6_bar", String, queue_size=Constants.QUEUE_SIZE)
    test_counter3_foo_pub = rospy.Publisher("/mission_control/test/variable/test_counter3_foo", String, queue_size=Constants.QUEUE_SIZE)
    test_default_pub = rospy.Publisher("/mission_control/test/variable/test_default", String, queue_size=Constants.QUEUE_SIZE)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha.spin()
        rate.sleep()

        if beha._token:
            #Own first state
            test_counter6_foo = beha.get_var('test_counter6_foo')
            test_counter6_foo_pub.publish(str(test_counter6_foo))

            #Own second state
            test_counter6_bar = beha.get_var('test_counter6_bar')
            test_counter6_bar_pub.publish(str(test_counter6_bar))

            #Other nodes
            test_counter3_foo = beha.get_var('test_counter3_foo')
            test_counter3_foo_pub.publish(str(test_counter3_foo))

            #Default value
            test_default = beha.get_var('some_random_val', 777)
            test_default_pub.publish(str(test_default))

    coverage_utils.cov_stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
