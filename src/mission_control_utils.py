import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from mission_control.msg import Variable

MAX_CBS = 4
"""int: how many times get_var functions can call itself """

VAR_RECHECK_DELAY = 0.5
"""float: how long function sleeps, before it goes into recursion """

VAR_GET_TOPIC = "/mission_control/variable/get"
"""string: topic name for getting variables """

VAR_SET_TOPIC = "/mission_control/variable/set"
"""string: topic name for setting variables """

QUEUE_SIZE = 10
"""int: determines queue size for rospy publishers """

cch = {}
"""dict: variable cache for scripts """

set_pub = rospy.Publisher(VAR_SET_TOPIC, Variable, queue_size=QUEUE_SIZE)
"""rospy.Publisher: publisher which sends out messages for setting variables """

get_pub = rospy.Publisher(VAR_GET_TOPIC, String, queue_size=QUEUE_SIZE)
"""rospy.Publisher: publisher which sends out messages for getting variables """

set_sub = False
"""rospy.Subscriber: subscriber which receives messages for getting variables """

def set_var_cb(data):
    """ Deals with incoming set variable msg
    
    If variable exists in cache then the value is updated otherwise it's ignored
    This callback is used only if script calls out mission_control_utils.ros_init 
    to make it into separate ROS node
    
    Args:
        data (mission_control.msg.Variable): data.name is the variable's name, data.value is the variable's value
    """

    global cch

    if data.name in cch or ("_" + data.name) in cch:
        cch[data.name] = data.value


def set_var(name, value):
    """ Sends out message to all listening nodes about new variable value.

    Args:
        name (string): variable's name
        value (mixed): variable's value
    """

    global set_pub

    msg = Variable()
    msg.name = str(name)
    msg.value = str(value)
    set_pub.publish(msg)


def get_var(name, def_val=None, counter=0):
    """ Request given variable's value

    First checks cache, if variable exists there.
    Lastly, if variable is not in cache, requests variable from all initialized nodes

    Args:
        name (string): requested variable's name
        def_val (mixed): default value when requested variabele is not found
        counter (int): number of times function is in recursion

    Returns:
        mixed: requested variable's value
    """

    global cch, VAR_RECHECK_DELAY, MAX_CBS

    if name in cch:
        return cch[name]

    if counter == 0:
        cch['_'+name] = True

    publish_get_var(name)

    if counter > MAX_CBS:
        cch[name] = def_val
        return def_val
 
    counter += 1

    time.sleep(VAR_RECHECK_DELAY)

    return get_var(name, def_val, counter)


def publish_get_var(name):
    """ Sends out message to all listening nodes about variable request.

    Args:
        name (string): variable's name
    """

    global get_pub

    get_pub.publish(name)


def ros_init(node_name):
    """ Initializes ROS node and necessary publishers/subscribers for scripts 
    
    Args:
        node_name (string): name for the created node
    """

    global VAR_SET_TOPIC, VAR_GET_TOPIC, set_pub, get_pub, set_sub

    rospy.init_node(node_name, anonymous=True)

    set_pub = rospy.Publisher(VAR_SET_TOPIC, Variable, queue_size=QUEUE_SIZE)
    get_pub = rospy.Publisher(VAR_GET_TOPIC, String, queue_size=QUEUE_SIZE)

    set_sub = rospy.Subscriber(VAR_SET_TOPIC, Variable, set_var_cb)
