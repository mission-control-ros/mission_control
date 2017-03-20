#!/usr/bin/env python
import rospy
import watchdog

def main():
    rospy.init_node('watchdog', anonymous=True)

    watch = watchdog.Watchdog(rospy.get_param('~node_dead_after', 2), rospy.get_param('~debug_file', ''))
    watch.set_debug(rospy.get_param('~debug', False))

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        watch.check_nodes()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
