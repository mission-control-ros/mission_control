#!/usr/local/bin/python
import coverage_utils
coverage_utils.cov_start()

import time
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import sys
import rospkg
import rospy

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import mission_control_utils
from mission_control_utils import Constants

mission_control_utils.ros_init("custom_script_priority6_get")

answer_pub = rospy.Publisher("/mission_control/test/mission_control_utils/answer_test_counter6", Int32, queue_size=Constants.QUEUE_SIZE)
expired_answer_pub = rospy.Publisher("/mission_control/test/mission_control_utils/expired_answer_test_counter6", Int32, queue_size=Constants.QUEUE_SIZE)
start_setting_pub = rospy.Publisher("/mission_control/test/mission_control_utils/start_setting", Bool, queue_size=Constants.QUEUE_SIZE)

time.sleep(2)

start_setting_pub.publish(True)

answer = mission_control_utils.get_var("test_counter6", -1)

answer_pub.publish(int(answer))

time.sleep(10) #Expire var

expired_answer = mission_control_utils.get_var("test_counter6", -1)

expired_answer_pub.publish(expired_answer)

coverage_utils.cov_stop()
