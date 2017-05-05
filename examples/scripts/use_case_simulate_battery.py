#!/usr/bin/env python
import rospy
import use_case_battery

def main():
    rospy.init_node('battery', anonymous=True)

    battery = use_case_battery.Battery()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        battery.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
