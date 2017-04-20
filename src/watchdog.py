import logging
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from mission_control_utils_constants import Constants
from mission_control.msg import Health

class Watchdog:

    ACTIVATE_TOPIC = "/mission_control/watchdog/fail_safe/activate"

    WATCHDOG_OK_TOPIC = "/mission_control/watchdog/ok"
    """string: node alive topic name """

    _node_dead_after = None
    """rospy.Duration: shows the duration which has to pass, when node is declared dead """

    _debug = False
    """bool: when True extra information is printed out """

    _logger = None
    """logging.Logger: logger object for logging info into log file """

    _nodes = {}
    """dict: holds all the nodes that are being monitored """

    _tokens = {}

    _all_ok = True

    activate_pub = rospy.Publisher(ACTIVATE_TOPIC, Bool, queue_size=Constants.QUEUE_SIZE)

    def __init__(self, dead_after=2, log_file_name=""):
        """ Class constructor

        Args:
            dead_after (int): shows the duration in seconds after what node is declared dead
            log_file_name (string): path to log file
        """

        self.subscribe_to_topics()
        self.set_node_dead_after(dead_after)
        self.init_logging(log_file_name)

    def set_debug(self, debug):
        """ Debug setter

        Args:
            debug (bool): when True extra info is showed
        """

        self._debug = debug

    def init_logging(self, log_file_name):
        """ Initializes logger

        Args: 
            log_file_name (string): file name where log info is written
        """

        self._log_file_name = log_file_name

        logger = logging.getLogger('watchdog')
        hdlr = logging.FileHandler(log_file_name, mode="w")
        formatter = logging.Formatter('[%(asctime)s]:%(message)s')
        hdlr.setFormatter(formatter)
        logger.addHandler(hdlr) 
        logger.setLevel(logging.WARNING)
        self._logger = logger

    def set_node_dead_after(self, seconds):
        """ Initializes _node_dead_after

        Args: 
            seconds (int): time in seconds after which node is declared dead
        """

        if seconds > 0:
            self._node_dead_after = rospy.Duration.from_sec(seconds)

    def subscribe_to_topics(self):
        """ Subscribes to all the needed topics """

        rospy.Subscriber(self.WATCHDOG_OK_TOPIC, Health, self.node_ok_cb)

    def log_warning(self, msg):
        """ Logs message through rospy.logwarn and also if logger is set, writes into log file

        Args:
            msg (string): warning message to be logged
        """

        rospy.logwarn(msg)

        if self._logger != None:
            self._logger.warning(msg)
       

    def node_ok_cb(self, data):
        """ Registers node's last alive time

        Args:
            data (std_msgs.msg.String): node's name
        """

        name = data.node_name
        token = data.token
        self._nodes[name] = rospy.Time.now()
        self._tokens[name] = token

    def check_nodes(self):
        """ Checks all the registered nodes and reports their status """

        if self._node_dead_after == None:
            self.log_warning("Time after which node is declared dead is not set!")

        token = False

        for name in self._nodes:
            last_time = self._nodes[name]

            if last_time + self._node_dead_after < rospy.Time.now():
                self.log_warning(name + " is dead")
                self._all_ok = False
                token = token or self._tokens[name] # If some of the dead nodes had the token, we have to give fail safe the token from the beginning

        if not self._all_ok:
            self.activate_pub.publish(token)

        if self._all_ok and self._debug:
            rospy.loginfo("All nodes are working")
