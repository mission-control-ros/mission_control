#!/usr/bin/env python
import rospy
import behaviour

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

    rospy.on_shutdown(beha.kill_process)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
