#!/usr/bin/env python
import rospy
import behaviour_fail_safe

def main():
    rospy.init_node('behaviour_fail_safe', anonymous=True)
    beha_fail_safe = behaviour_fail_safe.Behaviour_Fail_Safe()
    beha_fail_safe.set_debug_level(rospy.get_param('~debug', 0))
    beha_fail_safe.set_executable(rospy.get_param('~state_machine'))

    """
    We need to make a little pause after start-up so some of the token release/request 
    do not go missing
    """
    rospy.sleep(float(rospy.get_param('~wait_before_startup', 1)))

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha_fail_safe.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
