#!/usr/bin/env python
import rospy
import behaviour
from std_msgs.msg import String

def main():
    rospy.init_node('behaviour', anonymous=True)
    token = rospy.get_param('~token');
    beha = behaviour.Behaviour()
    beha.set_priority(rospy.get_param('~priority'))
    beha.set_active(rospy.get_param('~active'))
    beha.set_executable(rospy.get_param('~statemachine'))

    """
    We need to make a little pause after start-up so some of the token release/request 
    do not go missing
    """
    rospy.sleep(float(rospy.get_param('~wait_before_startup', 1)))

    if token:
        beha.set_token()

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
