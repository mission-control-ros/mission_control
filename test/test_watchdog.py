#!/usr/bin/env python
import sys
import os
import rospy
import time
import rospkg
import unittest

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

from mission_control_utils_constants import Constants
from std_msgs.msg import Bool
from mission_control.msg import Health

class TestWatchdog(unittest.TestCase):

    msg_got = False
    watchdog_ok = False

    def callback(self, data):
        self.msg_got = True
        self.watchdog_not_ok = data.data

    def test_token_on_startup(self):
        rospy.init_node('test_watchdog', anonymous=True)
        rospy.Subscriber("/mission_control/test/watchdog/node_not_ok", Bool, self.callback)
        ok_pub = rospy.Publisher("/mission_control/watchdog/ok", Health, queue_size=Constants.QUEUE_SIZE)

        msg = Health()
        msg.node_name = rospy.get_name()
        msg.token = False

        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.msg_got and time.time() < timeout_t:
            ok_pub.publish(msg)
            time.sleep(0.1)
        
        self.assertTrue(self.watchdog_not_ok)
        self.msg_got = False

        time.sleep(5)

        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.msg_got and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertFalse(self.watchdog_not_ok)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('mission_control', 'test_watchdog', TestWatchdog)
