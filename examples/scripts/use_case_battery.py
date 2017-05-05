import rospy
from std_msgs.msg import Bool
import rospkg
import sys

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import mission_control_utils

class Battery:

    def __init__(self):
        self.battery_level = 100
        rospy.Subscriber("/mission_control/use_case/battery/recharge", Bool, self.recharge_cb)

    def recharge_cb(self, data):
        if self.battery_level <= 95:
            self.battery_level += 5
        else:
            self.battery_level = 100

    def spin(self):
        if self.battery_level > 0:
            self.battery_level -= 1
        #rospy.loginfo("Battery level in battery " + str(self.battery_level))
        mission_control_utils.set_var('battery_level', self.battery_level)
