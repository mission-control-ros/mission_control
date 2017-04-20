import rospy
import behaviour_subprocess
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String
from mission_control.msg import Variable
from mission_control.msg import Answer
from mission_control_utils_constants import Constants
import mission_control_utils

class Behaviour_Subprocess_Fail_Safe(behaviour_subprocess.Behaviour_Subprocess):

    ACTIVATE_TOPIC = "/mission_control/watchdog/fail_safe/activate"

    _active = False
    _activated_once = False

    def subscribe_to_topics(self):

        rospy.Subscriber(self.TOKEN_RELEASE_TOPIC, Int32, self.release_token_cb)

        rospy.Subscriber(Constants.VAR_SET_TOPIC, Variable, self.set_variable_cb)
        rospy.Subscriber(Constants.VAR_GET_TOPIC, String, self.get_variable_cb)

        rospy.Subscriber(self.ACTIVATE_TOPIC, Bool, self.set_fail_safe_activated)

    def set_fail_safe_activated(self, data):
        
        if not self._activated_once:
            self._active = True
            self._activated_once = True
            self._token = data.data

    def set_variable_cb(self, data):
        """ Deals with incoming set variable msg

        If variable exists in cache then the value is updated otherwise it's ignored

        Args:
            data (mission_control.msg.Variable): data.name is the variable's name, data.value is the variable's value, data.ttl is variable's validity
        """

        self._cache[data.name] = data.value

        self.write_debug("Setting variable named " + data.name + " with value " + str(data.value), 2)

    def get_variable_cb(self, data):
        """ Deals with get variable msg

        Searches current node's state machine for the given variable
        If variable is found, the sends out set variable msg

        Args:
            data (std_msgs.msg.String): variable name that is being searched
        """

        if self._active and data.data in self._cache:
            var = self._cache[data.data]
            mission_control_utils.set_var(data.data, var)



    def is_active(self):
        """ Return boolean value which shows whether the node is active or not """

        self.write_debug("Fail safe is " + str(self._active), 3)

        return self._active

    def spin(self):
        """ Checks if the node is active

        If the node is active and not running, activates node
        If the thread ends, deactivates node

        On every call sends a message that shows that the node is alive

        On startup asks for token as many times as it is defined in variable self.MAX_TOKEN_ASK_TIMES
        """

        active = self.is_active()

        if active: # Always ask for token to stop all other nodes
            self.request_token()

        if active and self._token and not self._running:
            self.activate()
        elif self._token and self._running and not self.is_thread_alive():
            self.deactivate()
            self._active = False
