import rospy
from mission_control_utils_constants import Constants
from std_msgs.msg import String
from mission_control.msg import Variable


class Cache:

    cch = {}
    """dict: variable cache for scripts"""

    ttl = {}
    """dict: time to live for scipts variables"""

    last_upt = {}
    """dict: last time scripts variables were updated"""

    set_pub = rospy.Publisher(Constants.VAR_SET_TOPIC, Variable, queue_size=Constants.QUEUE_SIZE)
    """rospy.Publisher: publisher which sends out messages for setting variables"""

    get_pub = rospy.Publisher(Constants.VAR_GET_TOPIC, String, queue_size=Constants.QUEUE_SIZE)
    """rospy.Publisher: publisher which sends out messages for getting variables"""

    set_sub = False
    """rospy.Subscriber: subscriber which receives messages for setting variables"""

    debug_level = -1
    """int: node's debug level whom's script or object uses this cache"""

    parent_node_name = ""
    """string: node's name whom's script or object uses this cache"""
