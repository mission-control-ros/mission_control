#!/usr/bin/env python
import rospy
import rospkg
import sys
rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import behaviour

def main():
    rospy.init_node('failing_behaviour', anonymous=True)
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

    rospy.on_shutdown(beha.kill_process)

    fail_counter = 0

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        if fail_counter < 10:
            beha.spin()
        else:
            rospy.signal_shutdown("Need to test fake node crash")

        rate.sleep()

        fail_counter += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
