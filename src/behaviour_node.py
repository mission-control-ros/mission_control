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
    beha.set_executable('')

    wait_rate = rospy.Rate(0.7)
    wait_rate.sleep()

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
