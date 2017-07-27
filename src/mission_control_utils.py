import rospy
import time
import sys
from std_msgs.msg import String
from std_msgs.msg import Bool
from mission_control.msg import Variable
from mission_control_utils_cache import Cache
from mission_control_utils_constants import Constants

def set_var_cb(data):
    """ Deals with incoming set variable msg
    
    If variable exists in cache then the value is updated otherwise it's ignored
    This callback is used only if script calls out mission_control_utils.ros_init 
    to make it into separate ROS node
    
    Args:
        data (mission_control.msg.Variable): data.name is the variable's name, data.value is the variable's value, data.ttl is variable's validity
    """

    if data.name in Cache.cch or ("_" + data.name) in Cache.cch:
        Cache.cch[data.name] = data.value
        Cache.last_upt[data.name] = rospy.Time.now()

        write_debug("Setting variable named " + data.name + " with value " + str(data.value), 2)

        if data.ttl > 0:
            Cache.ttl[data.name] = rospy.Duration.from_sec(data.ttl)
            write_debug("Setting variable named " + data.name + " with time to live " + str(data.ttl), 3)

def set_var(name, value, ttl = None):
    """ Sends out message to all listening nodes about new variable value.

    Args:
        name (string): variable's name
        value (mixed): variable's value
        ttl (int): variable's validity in seconds
    """

    msg = Variable()
    msg.name = str(name)
    msg.value = str(value)
    if ttl == None:
        msg.ttl = 0
    else:
        msg.ttl = int(ttl)

    msg.node_name = Cache.parent_node_name

    Cache.set_pub.publish(msg)

    write_debug("Publishing variable named " + msg.name + " with value " + str(msg.value) + " with time to live " + str(msg.ttl), 3)


def get_var(name, def_val=None, counter=0):
    """ Request given variable's value

    First checks cache, if variable exists there.
    Lastly, if variable is not in cache, requests variable from all initialized nodes

    Args:
        name (string): requested variable's name
        def_val (mixed): default value when requested variable is not found
        counter (int): number of times function is in recursion

    Returns:
        mixed: requested variable's value
    """

    if counter == 0:
        check_var(name)

    if name in Cache.cch:
        write_debug("Found variable named " + name + " in cache", 2)
        return Cache.cch[name]

    if counter == 0:
        Cache.cch['_'+name] = True

    publish_get_var(name)

    if counter > Constants.MAX_CBS:
        write_debug("Maximum callbacks for get_var function reached, setting variable named " + name + " with default value " + str(def_val), 2)
        Cache.cch[name] = def_val
        Cache.last_upt[name] = rospy.Time.now()
        return def_val
 
    counter += 1

    write_debug("Asking for variable named " + name + " (" + str(counter) + "/" + str(Constants.MAX_CBS) + ")", 3)

    time.sleep(Constants.VAR_RECHECK_DELAY)

    return get_var(name, def_val, counter)

def check_var(name):
    """Checks if the variable in cache is new enough or needs to be deleted

    Args:
        name (string): variable's name 
    """

    if name not in Cache.ttl or name not in Cache.last_upt:
        return

    var_ttl = Cache.ttl[name]
    var_last_upt = Cache.last_upt[name]

    if rospy.Time.now() > (var_last_upt + var_ttl):
        write_debug("Deleting variable named " + name + " from cache", 1)

        if name in Cache.cch:
            del Cache.cch[name]
            write_debug("Deleting variable named " + name + " from Cache.cch", 3)

        if "_"+name in Cache.cch:
            del Cache.cch["_" + name]
            write_debug("Deleting variable named _" + name + " from Cache.cch", 3)

        if name in Cache.ttl:
            del Cache.ttl[name]
            write_debug("Deleting time to live for variable named " + name + " from Cache.ttl", 3)

        if name in Cache.last_upt:
            del Cache.last_upt[name]
            write_debug("Deleting last update time for variable named " + name + " from Cache.last_upt", 3)

def publish_get_var(name):
    """ Sends out message to all listening nodes about variable request.

    Args:
        name (string): variable's name
    """

    Cache.get_pub.publish(name)

    write_debug("Publishing getting message for variable named " + name, 3)


def write_debug(msg, level):
    """ Writes node's script/object debug message

    Args:
        msg (string): debug message to be printed
        level (int): debug message's level
    """

    if Cache.debug_level >= level:
        rospy.loginfo(Cache.parent_node_name + " script/object - " + msg)


def subscribe_to_topics():
    """ Subscribe to all the topics needed in mission_control_utils """
    Cache.set_pub = rospy.Publisher(Constants.VAR_SET_TOPIC, Variable, queue_size=Constants.QUEUE_SIZE)
    Cache.get_pub = rospy.Publisher(Constants.VAR_GET_TOPIC, String, queue_size=Constants.QUEUE_SIZE)
    Cache.set_sub = rospy.Subscriber(Constants.VAR_SET_TOPIC, Variable, set_var_cb)

#This function is meant to use in custom scripts
def ros_init(node_name):
    """ Initializes ROS node and necessary publishers/subscribers for scripts 
    
    Args:
        node_name (string): name for the created node
    """

    if len(sys.argv) == 3:
        Cache.debug_level = int(sys.argv[1])
        Cache.parent_node_name = str(sys.argv[2])

    rospy.init_node(node_name, anonymous=True)

    subscribe_to_topics()

    write_debug("ROS init", 1)
