import rospy
import smach
import smach_ros
import time
import imp
import uuid
import subprocess
import signal
import os
import mission_control_utils
from mission_control_utils_constants import Constants
from mission_control_utils_cache import Cache
from std_msgs.msg import Int32
from std_msgs.msg import String
from mission_control.msg import Variable
from mission_control.msg import Answer
from mission_control.msg import Health

class Behaviour:

    TOKEN_REQUEST_TOPIC = "/mission_control/token/request"
    """string: token request topic name"""

    TOKEN_RELEASE_TOPIC = "/mission_control/token/release"
    """string: token release topic name"""

    TOKEN_ASK_TOPIC = "/mission_control/token/ask"
    """string: token ask topic name"""

    TOKEN_ANSWER_TOPIC = "/mission_control/token/answer"
    """string: token answer topic"""

    WATCHDOG_OK_TOPIC = "/mission_control/watchdog/ok"
    """string: watchdog ok topic name"""

    MAX_TOKEN_ASK_TIMES = 2
    """int: how many times node will ask for token on startup"""

    MAX_VAR_SEARCH_RECURSION = 10

    _token_ask_times = 0
    """int: how many times node has asked for token on startup"""

    _answer_false_gotten = False
    """bool: indicates whether node has been denied for startup token for even once"""

    _answer = True
    """bool: indicates whether node has the right to acquire token on startup.
    It is initialized to True, to solve the problem when only one node exists.
    """

    _priority = 0
    """int: node's priority. The lower the number the higher the priority

    Available values: 1..N
    """

    _active_str = "False"
    """string: after evaluation shows if the node is active or not """

    _token = False
    """bool: shows if other initialized nodes have given this node permission to work"""

    _paused = False
    """bool: shows if state machine's work is paused or not"""

    _running = False
    """bool: shows if the state machine is active or not"""

    _process = False
    """subprocess.Popen: subprocess where script will be executed """

    _script = ''
    """string: script's full path, that will be executed """

    _cache = {}
    """dict: all the variables that node uses from state machines are stored here"""

    _var_ttl = {}
    """dict: holds variables time to live in seconds"""

    _var_last_upt = {}
    """dict: holds the info when the variable was last set"""

    _vars_from_own_proc = {}
    """dict: holds all the variables that the child process set"""

    explored = []
    """list: all the objects in child python script that have been searched for variables"""

    types = []
    """list: all user defined class types that are in child python script"""

    _debug_level = 0
    """int: node's debug level between 0..3 . 0 - lowest level, 3 - highest level"""

    request_pub = rospy.Publisher(TOKEN_REQUEST_TOPIC, Int32, queue_size=Constants.QUEUE_SIZE)
    """rospy.Publisher: token request publisher"""

    release_pub = rospy.Publisher(TOKEN_RELEASE_TOPIC, Int32, queue_size=Constants.QUEUE_SIZE)
    """rospy.Publisher: token release publisher"""

    ok_pub = rospy.Publisher(WATCHDOG_OK_TOPIC, Health, queue_size=Constants.QUEUE_SIZE)
    """rospy.Publisher: watchdog ok publisher"""

    ask_pub = rospy.Publisher(TOKEN_ASK_TOPIC, Int32, queue_size=Constants.QUEUE_SIZE)
    """rospy.Publisher: startup token ask publisher"""

    answer_pub = rospy.Publisher(TOKEN_ANSWER_TOPIC, Answer, queue_size=Constants.QUEUE_SIZE)
    """rospy.Publisher: startup token answer publisher"""

    def __init__(self):

        Cache.parent_node_name = rospy.get_name()

        self.subscribe_to_topics()

    def set_debug_level(self, debug_level):
        """ Sets node's debug level

        Args:
            debug_level (int): debug level
        """

        if debug_level < 0:
            debug_level = 0
        elif debug_level > 3:
            debug_level = 3

        Cache.debug_level = debug_level
        self._debug_level = debug_level

    def set_priority(self, prio):
        """ Sets node's priority 

        Args:
            prio (int): priority number
        """

        if prio < 1:
            self._priority = 1
        else:
            self._priority = prio

    def set_executable(self, file):
        """ Sets executable script 
        
        Args:
            file (string): file's name, that will be executed
        """
        self._script = file

        return True

    def subscribe_to_topics(self):
        """ Subscribes to all ROS topics """

        rospy.Subscriber(self.TOKEN_REQUEST_TOPIC, Int32, self.request_token_cb)
        rospy.Subscriber(self.TOKEN_RELEASE_TOPIC, Int32, self.release_token_cb)

        rospy.Subscriber(self.TOKEN_ASK_TOPIC, Int32, self.ask_token_cb)
        rospy.Subscriber(self.TOKEN_ANSWER_TOPIC, Answer, self.answer_token_cb)

        rospy.Subscriber(Constants.VAR_SET_TOPIC, Variable, self.set_variable_cb)
        rospy.Subscriber(Constants.VAR_GET_TOPIC, String, self.get_variable_cb)
    

    def set_active(self, active_str):
        """ Sets string that is later evaluated to boolean value which shows whether the node should be active

        Args:
            active_str (string): string that will be evaluated
        """

        if active_str != "":
            self._active_str = active_str

    def is_active(self):
        """ Return boolean value which shows whether the node is active or not """

        active = eval(self._active_str)

        self.write_debug("Activate string " + self._active_str + " evaluates to " + str(active), 3)

        return active

    def request_token(self):
        """ Sends out token request message """

        self.request_pub.publish(self._priority)

        self.write_debug("Requesting token", 2)

    def release_token(self, rel_prio):
        """ Releases held token

        If necessary pauses the current state machine

        Args:
            rel_prio (int): indicates to which priority token is released to
        """

        if self.is_active():
            self.pause_behaviour()

        self._token = False
        self.release_pub.publish(rel_prio)

        self.write_debug("Releasing token to priority " + str(rel_prio), 2)

    def ask_token(self):
        """Asks for token on startup
        This is different from token request because token is asked on startup and nobody has token at startup.
        On startup token is given to node which has the highest priority.
        If some node asks for token when somebody alrady has token, false is returned despite its priority.

        Returns:
            bool: True if node is allowed to ask token again, False otherwise
        """

        if self._token_ask_times > self.MAX_TOKEN_ASK_TIMES:
            return False

        if self._token_ask_times == self.MAX_TOKEN_ASK_TIMES:
            if self._answer_false_gotten:
                self._token = False
                self.write_debug("Got False atleast at once on asking token", 2)
            else:
                self._token = self._answer
                self.write_debug("Got answer " + str(self._answer) + " when asking token on startup", 2)

            self._token_ask_times += 1

            return False

        if not self._answer_false_gotten:
            self.ask_pub.publish(self._priority)

        self._token_ask_times += 1

        self.write_debug("Asking token on startup (" + str(self._token_ask_times) + "/" + str(self.MAX_TOKEN_ASK_TIMES) + ")", 3)

        return True

    def answer_token_cb(self, data):
        """ Deals with incoming token answer message

        If node has been given false as an answer even once, it has no chance to acquire token on startup

        Args:
            data (mission_control.msg.Answer): data.priority shows node's priority to which this answer is meant to ,
        data.answer is the answer given to node
        """

        if self._priority == data.priority and not self._answer_false_gotten:
            self._answer = data.answer

            if not self._answer:
                self._answer_false_gotten = True

    def ask_token_cb(self, data):
        """Deals with incoming token ask message

        If node's priority, who asked, is higher or equal than the node's from who it's being asked, permission to acquire node is given.
        If node's priority is lower, permission to acquire node is denied

        If node, from who the token permission is asked, owns the token, permission to acquire node is denied
        """

        if data.data == self._priority:
            return

        msg = Answer()
        msg.priority = data.data

        if self._token or self._priority < data.data:
            msg.answer = False
        else: 
            msg.answer = True

        self.write_debug("Priority " + str(data.data) + " asked for token, got answer " + str(msg.answer), 3)

        self.answer_pub.publish(msg)

    def request_token_cb(self, data):
        """ Deals with incoming token request message

        Args:
            data (std_msgs.msg.Int32): priority number of the node who requested token
        """
        
        if self._token and (data.data < self._priority or not self.is_active()):
            self.release_token(data.data)

    def release_token_cb(self, data):
        """ Deals with incoming token release message.

        If necessary claims token and resumes current state machine.

        Args:
            data (std_msgs.msg.Int32): priority number of the node to whom token was released
        """
        
        if data.data != self._priority:
            return
        
        self._token = True
        self.write_debug("Got token", 1)
        
        if self._paused:
            self.resume_behaviour()

    def pause_behaviour(self):
        """ Pauses the current subprocess """

        os.kill(self._process.pid, signal.SIGSTOP)
        self._paused = True

        self.write_debug("Pausing node", 1)

    def resume_behaviour(self):
        """ Resumes the current subprocess """

        os.kill(self._process.pid, signal.SIGCONT)
        self._paused = False

        self.write_debug("Resuming node", 1)

    def set_variable_cb(self, data):
        """ Deals with incoming set variable msg

        If variable exists in cache then the value is updated otherwise it's ignored

        Args:
            data (mission_control.msg.Variable): data.name is the variable's name, data.value is the variable's value, data.ttl is variable's validity
        """

        var_from_own_proc = data.node_name == rospy.get_name()

        if var_from_own_proc:
            self._vars_from_own_proc[data.name] = data.value

            self.write_debug("Setting variable named " + data.name + " with value " + str(data.value) + " as own variable", 2)

        if data.name in self._cache or ("_" + data.name) in self._cache or var_from_own_proc:
            self._cache[data.name] = data.value
            self._var_last_upt[data.name] = rospy.Time.now()

            self.write_debug("Setting variable named " + data.name + " with value " + str(data.value), 2)

            if data.ttl > 0:
                self._var_ttl[data.name] = rospy.Duration.from_sec(data.ttl)
                self.write_debug("Setting variable named " + data.name + " with time to live " + str(data.ttl), 3)

    def get_variable_cb(self, data):
        """ Deals with get variable msg

        If child process isn't running then node tries to search for the variable
        in child python script.

        Otherwise looks, if the requested variable has been set by the child process.

        If variable is found by either of these methods, then the variable is set
        using mission_control_utils' function set_var

        Args:
            data (std_msgs.msg.String): variable name that is being searched
        """

        self.check_var(data.data)

        if os.path.isfile(self._script) and self.node_not_running():
            found, var = self.find_var(data.data)

            if found:
                mission_control_utils.set_var(data.data, var)
                return

        elif data.data in self._vars_from_own_proc:
            ttl = None

            if data.data in self._var_ttl:
                ttl = self._var_ttl[data.data].to_sec()

            mission_control_utils.set_var(data.data, self._vars_from_own_proc[data.data], ttl)

    def find_var(self, var_name):
        """ Tries to find the requested variable from child python file
        that is stored in seld._script variable

        Args:
            var_name (string): variable name that is being searched

        Returns:
            bool: True if variable was found, False otherwise
            mixed: None, if the variable wasn't found, otherwise variable's value
        """

        found = False
        variable = None
        self.explored = []
        self.types = [smach.StateMachine]

        try:
            state_machine = imp.load_source(str(uuid.uuid1()), self._script)
        except:
            return found, variable

        for var in  dir(state_machine):
            obj = eval('state_machine.' + var)

            if isinstance(obj, type) and obj not in self.types:
                self.types.append(obj)

        for var in  dir(state_machine):
            obj = eval('state_machine.' + var)

            if isinstance(obj, tuple(self.types)):
                found, variable = self.explore_obj(obj, var_name)

            if found:
                return found, variable

        return found, variable

    def explore_obj(self, obj, var_name, cnt=0):
        """ Explores all objects variables for the requested variable

        Args:
            obj (mixed): some user defined class or smach.Statemachine class
            var_name (string): variable name that is being searched
            cnt (int): how deep the recursion is

        Returns:
            bool: True if variable was found, False otherwise
            mixed: None, if the variable wasn't found, otherwise variable's value
        """

        if obj in self.explored or cnt > self.MAX_VAR_SEARCH_RECURSION:
            return False, None

        found = False
        found_var = None

        self.explored.append(type(obj))

        for var in dir(obj):
            var_obj = eval('obj.' + var)
            var_type = type(var_obj)

            if var == var_name and var_type in (bool, str, int, long, float):
                return True, var_obj

            if var_type is dict:
                found, found_var = self.explore_dict(var_obj, var_name, cnt+1)

            if var_type in (list, tuple):
                found, found_var = self.explore_list(var_obj, var_name, cnt+1)

            if isinstance(var_obj, tuple(self.types)):
                found, found_var = self.explore_obj(obj, var_name, cnt+1)

            if found:
                return found, found_var

        return found, found_var

    def explore_dict(self, dct, var_name, cnt):
        """ Searches dictionary for any objects that could be further explored

        Args:
            dct (dict): dictionary to be searched
            var_name (string): variable name that is being searched
            cnt (int): how deep the recursion is

        Returns:
            bool: True if variable was found, False otherwise
            mixed: None, if the variable wasn't found, otherwise variable's value
        """

        found = False
        found_var = None

        if cnt > self.MAX_VAR_SEARCH_RECURSION:
            return found, found_var

        for key in dct:
            obj = dct[key]
            if isinstance(obj, tuple(self.types)):
                found, found_var = self.explore_obj(obj, var_name, cnt+1)

            if found:
                return found, found_var

        return found, found_var

    def explore_list(self, lst, var_name, cnt):
        """ Searches list for any objects that could be further explored

        Args:
            lst (lst): list to be searched
            var_name (string): variable name that is being searched
            cnt (int): how deep the recursion is

        Returns:
            bool: True if variable was found, False otherwise
            mixed: None, if the variable wasn't found, otherwise variable's value
        """

        found = False
        found_var = None

        if cnt > self.MAX_VAR_SEARCH_RECURSION:
            return found, found_var

        for i in range(0, len(lst)):
            obj = lst[i]
            if isinstance(obj, tuple(self.types)):
                found, found_var = self.explore_obj(obj, var_name, cnt+1)

            if found:
                return found, found_var

        return found, found_var

    def get_var(self, name, def_val=None, counter=0):
        """ Request given variable's value

        First: checks cache, if variable exists there
        Second: if variable is not in cache then searches for it in child python file if the process isn't running
        Third: lastly, if variable is not in node's statemachine, requests variable from all initialized nodes

        Args:
            name (string): requested variable's name

        Returns:
            mixed: requested variable's value
        """

        if counter == 0:
            self.check_var(name)

        if name in self._cache:
            self.write_debug("Found variable named " + name + " in cache", 2)
            return self._cache[name]

        if os.path.isfile(self._script) and self.node_not_running:
            found, var = self.find_var(name)

            if found:
                self._cache[name] = var
                return var

        if counter == 0:
            self._cache["_"+name] = True
            mission_control_utils.publish_get_var(name)

        if counter > Constants.MAX_CBS:
            self.write_debug("Maximum callbacks for get_var function reached, setting variable named " + name + " with default value " + str(def_val), 2)

            self._cache[name] = def_val
            return def_val

        counter += 1

        self.write_debug("Asking for variable named " + name + " (" + str(counter) + "/" + str(Constants.MAX_CBS) + ")", 3)

        time.sleep(Constants.VAR_RECHECK_DELAY)

        return self.get_var(name, def_val, counter)


    def check_var(self, name):
        """Checks if the variable in cache is new enough or needs to be deleted

        Args:
            name (string): variable's name
        """
        
        if name not in self._var_ttl or name not in self._var_last_upt:
            return

        var_ttl = self._var_ttl[name]
        var_last_upt = self._var_last_upt[name]

        if rospy.Time.now() > (var_last_upt + var_ttl):
            self.write_debug("Deleting variable named " + name + " from cache", 1)

            if name in self._cache:
                del self._cache[name]
                self.write_debug("Deleting variable named " + name + " from self._cache", 3)

            if "_"+name in self._cache:
                del self._cache["_" + name]
                self.write_debug("Deleting variable named _" + name + " from self._cache", 3)

            if name in self._vars_from_own_proc:
                del self._vars_from_own_proc[name]
                self.write_debug("Deleting variable named _" + name + " from self._vars_from_own_proc", 3)

            if name in self._var_ttl:
                del self._var_ttl[name]
                self.write_debug("Deleting time to live for variable named " + name + " from self._var_ttl", 3)

            if name in self._var_last_upt:
                del self._var_last_upt[name]
                self.write_debug("Deleting last update time for variable named " + name + " from self._var_last_upt", 3)

    def activate(self):
        """ Activates node """

        if(os.path.isfile(self._script)):
            cmd = "exec python %s %d %s"
        else:
            cmd = "exec rosrun %s %d %s"

        cmd = cmd % (self._script, self._debug_level, rospy.get_name())

        self._process = subprocess.Popen(cmd,shell=True)
        self._running = True

        self.write_debug("Node activates", 1)

    def deactivate(self):
        """ Deactivates node """

        self._running = False
        self._process = False

        self.write_debug("Node deactivates", 1)

    def node_not_running(self):

        return not self._running and not self._process

    def kill_process(self):
        """ Kills the active process """

        if self._process:
            self._process.kill()

    def is_subprocess_alive(self):
        """ Checks if the subprocess, in which the script runs, is alive 

        Returns:
            bool: True if subprocess is alive, False otherwise
        """

        if not self._process:
            return False

        self._process.poll()
        alive = self._process.returncode == None

        self.write_debug("Subprocess is " + str(alive), 3)

        return alive

    def node_is_ok(self):
        """ Publishes message that shows that node is alive """

        msg = Health()
        msg.node_name = rospy.get_name()
        msg.token = self._token

        self.ok_pub.publish(msg)

        self.write_debug("Publishing node is ok message", 3)
        

    def spin(self):
        """ Checks if the node is active

        If the node is active and not running, activates node
        If the thread ends, deactivates node

        On every call sends a message that shows that the node is alive

        On startup asks for token as many times as it is defined in variable self.MAX_TOKEN_ASK_TIMES
        """

        self.node_is_ok()

        allowed_to_ask_token = self.ask_token()

        self.write_debug("mul on token: " + str(self._token), 3)

        if allowed_to_ask_token:
            self.write_debug("Node is not allowed to ask token", 2)
            return

        active = self.is_active()

        if active and not self._token:
            self.request_token()

        if active and self._token and not self._running:
            self.activate()
        elif self._token and self._running and not self.is_subprocess_alive():
            self.deactivate()

    def write_debug(self, msg, level):
        """ Writes node's debug message

        Args:
            msg (string): debug message to be printed
            level (int): debug message's level
        """

        if self._debug_level >= level:
            rospy.loginfo(rospy.get_name() + " - " + msg)
