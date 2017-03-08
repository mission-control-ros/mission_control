import rospy
import smach
import smach_ros
import time
import mission_control_utils
from std_msgs.msg import Int32
from std_msgs.msg import String
from mission_control.msg import Variable
from threading import Thread
import imp

class Behaviour:

    TOKEN_REQUEST_TOPIC = "/mission_control/token/request"
    """string: token request topic name """

    TOKEN_RELEASE_TOPIC = "/mission_control/token/release"
    """string: token release topic name """

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

    _run_thread = False
    """thread.Thread: thread, in which the state machine is working"""

    _sm = False
    """statemachine.Statemachine: current nodes state machine"""

    _cache = {}
    """dict: all the variables that node uses from state machines are stored here"""

    request_pub = rospy.Publisher(TOKEN_REQUEST_TOPIC, Int32, queue_size=mission_control_utils.QUEUE_SIZE)
    """rospy.Publisher: token request publisher"""

    release_pub = rospy.Publisher(TOKEN_RELEASE_TOPIC, Int32, queue_size=mission_control_utils.QUEUE_SIZE)
    """rospy.Publisher: token release publisher"""

    def __init__(self):

        self.subscribe_to_topics()


    def set_token(self):
        self._token = True

    def set_priority(self, prio):
        """ Sets node's priority 

        Args:
            prio (int): priority number
        """

        self._priority = prio

    def set_executable(self, file):
        """ Sets state machine that will be executed 
        
        Args:
            file (string): path to python file where atleast one state machine is defined

        Returns:
            bool: True if StateMachine was successfully found, False otherwise
        """

        state_machine = imp.load_source('state_machine', file)
        for var_name in  dir(state_machine):
            var = eval('state_machine.' + var_name)

            if isinstance(var, smach.StateMachine):
                self._sm = var
                break

        if not self._sm:
            rospy.logfatal("Could not find StateMachine from file %s" % file)
            rospy.signal_shutdown("Could not find StateMachine from file %s" % file)
            return False

        func_type = type(self._sm._update_once)
        self._sm._old_update_once = self._sm._update_once
        self._sm._update_once = func_type(_new_update_once, self._sm, smach.StateMachine)
        self._sm._paused = False

        return True

    def subscribe_to_topics(self):
        """ Subscribes to all ROS topics """

        rospy.Subscriber(self.TOKEN_REQUEST_TOPIC, Int32, self.request_token_cb)
        rospy.Subscriber(self.TOKEN_RELEASE_TOPIC, Int32, self.release_token_cb)

        rospy.Subscriber(mission_control_utils.VAR_SET_TOPIC, Variable, self.set_variable_cb)
        rospy.Subscriber(mission_control_utils.VAR_GET_TOPIC, String, self.get_variable_cb)
    

    def set_active(self, active_str):
        """ Sets string that is later evaluated to boolean value which shows whether the node should be active

        Args:
            active_str (string): string that will be evaluated
        """

        if active_str != "":
            self._active_str = active_str

    def is_active(self):
        """ Return boolean value which shows whether the node is active or not """

        return eval(self._active_str)

    def request_token(self):
        """ Sends out token request message """

        self.request_pub.publish(self._priority)

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

    def request_token_cb(self, data):
        """ Deals with incoming token request message

        Args:
            data (std_msgs.msg.Int32): priority number of the node who requested token
        """
        
        if data.data == -1:
            self._token = False
        elif self._token and (data.data < self._priority or not self.is_active()):
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
        
        if self._paused:
            self.resume_behaviour()

    def pause_behaviour(self):
        """ Pauses the current state machine """

        self._sm._paused = True
        self._paused = True

    def resume_behaviour(self):
        """ Resumes the current state machine """

        self._sm._paused = False
        self._paused = False

    def set_variable_cb(self, data):
        """ Deals with incoming set variable msg

        If variable exists in cache then the value is updated otherwise it's ignored

        Args:
            data (mission_control.msg.Variable): data.name is the variable's name, data.value is the variable's value
        """

        if data.name in self._cache or ("_" + data.name) in self._cache:
            self._cache[data.name] = data.value

    def get_variable_cb(self, data):
        """ Deals with get variable msg

        Searches current node's state machine for the given variable
        If variable is found, the sends out set variable msg

        Args:
            data (std_msgs.msg.String): variable name that is being searched
        """

        found, var = self.get_var_from_sm(data.data)
        if found:
            mission_control_utils.set_var(data.data, var)

    def get_var_from_sm(self, name):
        """ Searches node's state machine for given variable

        Args:
            name (string): variable name which is being searched

        Returns:
            bool: True if variable found, False otherwise
            mixed: If variable found, then that variables value is returned, otherwise False is returned
        """

        states = self._sm.__dict__['_states']

        for state_name in states:
            state = states[state_name]
            if name in state.__dict__:
                return True, state.__dict__[name]
        return False, False
        

    def get_var(self, name, def_val=None, counter=0):
        """ Request given variable's value

        First: checks cache, if variable exists there
        Second: if variable is not in cache then searches variable from iteself's state machine
        Third: lastly, if variable is not in node's state machine, requests variable from all initialized nodes

        Args:
            name (string): requested variable's name
            def_val (mixed): default value when requested variabele is not found
            counter (int): number of times function is in recursion


        Returns:
            mixed: requested variable's value
        """

        if name in self._cache:
            return self._cache[name]
        
        if counter == 0:
            found, var_in_my_sm = self.get_var_from_sm(name)
           
            if found: #Because returned value can be whatever
                self._cache[name] = var_in_my_sm
                return var_in_my_sm

            self._cache["_"+name] = True
            mission_control_utils.publish_get_var(name)

        if counter > mission_control_utils.MAX_CBS:
            self._cache[name] = def_val
            return def_val

        counter += 1

        time.sleep(mission_control_utils.VAR_RECHECK_DELAY)

        return self.get_var(name, def_val, counter)

    def activate(self):
        """ Activates node """

        self._running = True

        self._run_thread = Thread(target=self._sm.execute)
        self._run_thread.start()

    def deactivate(self):
        """ Deactivates node """

        self._running = False

    def is_thread_alive(self):
        """ Checks if the thread, in which the state machine runs, is alive 

        Returns:
            bool: True if thread is alive, False otherwise
        """

        return self._run_thread.is_alive()

    def spin(self):
        """ Checks if the node is active

        If the node is active and not running, activates node
        If the thread ends, deactivates node
        """

        active = self.is_active()

        if active and not self._token:
            self.request_token()

        if active and self._token and not self._running:
            self.activate()
        elif self._token and self._running and not self.is_thread_alive():
            self.deactivate()

def _new_update_once(self):
    """ Overrides class' Statemachine function _update_once

    This is necessary so the state machine inside the node could be paused and later resumed

    """

    if not self._paused:
        return self._old_update_once()
    return None
