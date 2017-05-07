#!/usr/local/bin/python
import coverage_utils
coverage_utils.cov_start()

import time
from std_msgs.msg import Bool
import sys
import rospkg
import rospy

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import mission_control_utils
from mission_control_utils import Constants

mission_control_utils.ros_init("custom_script_priority6_1")

stop_asking = False

def stop_asking_callback(data):
    global stop_asking
    stop_asking = True

start_asking_pub = rospy.Publisher("/mission_control/test/mission_control_utils/start_asking", Bool, queue_size=Constants.QUEUE_SIZE)

rospy.Subscriber("/mission_control/test/mission_control_utils/stop_asking", Bool, stop_asking_callback)

test_counter6_1 = 15

time.sleep(5)

start_asking_pub.publish(True)

timeout_t = time.time() + 10.0

while not stop_asking and time.time() < timeout_t:
    print "Publishin"
    mission_control_utils.set_var("test_counter6_1", test_counter6_1)

    time.sleep(0.5)

coverage_utils.cov_stop()
