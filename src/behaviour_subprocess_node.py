#!/usr/bin/env python
import rospy
import behaviour_subprocess
from std_msgs.msg import String

def main():
    rospy.init_node('behaviour_subprocess', anonymous=True)
    token = rospy.get_param('~token');
    beha = behaviour_subprocess.Behaviour_Subprocess()
    beha.set_priority(rospy.get_param('~priority'))
    beha.set_active(rospy.get_param('~active'))
    beha.set_executable(rospy.get_param('~script'))

    if token:
        beha.set_token()

    wait_rate = rospy.Rate(0.7)
    wait_rate.sleep()

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        beha.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
