import rospy
import time
import mission_control_utils
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from mission_control.msg import Variable
import behaviour
import subprocess
import signal
import os

class Behaviour_Subprocess(behaviour.Behaviour):

    _process = False
    """subprocess.Popen: subprocess where script will be executed """

    _script = ''
    """string: script's full path, that will be executed """

    def set_executable(self, file):
        """ Sets executable script 
        
        Args:
            file (string): file's name, that will be executed
        """
        self._script = file

    def get_var(self, name, def_val=None, counter=0):
        """ Request given variable's value

        First: checks cache, if variable exists there
        Second: if variable is not in cache then searches variable from iteself's statemachine
        Third: lastly, if variable is not in node's statemachine, requests variable from all initialized nodes

        Args:
            name (string): requested variable's name

        Returns:
            mixed: requested variable's value
        """

        if name in self._cache:
            return self._cache[name]

        if counter == 0:
            self._cache["_"+name] = True
            mission_control_utils.publish_get_var(name)

        if counter > mission_control_utils.MAX_CBS:
            self._cache[name] = def_val
            return def_val

        counter += 1

        time.sleep(mission_control_utils.VAR_RECHECK_DELAY)

        return self.get_var(name, def_val, counter)

    def get_variable_cb(self, data):
        """ Deals with get variable msg

        Searches current node's cache for the variable
        If variable is found, the sends out set variable msg

        Args:
            data (std_msgs.msg.String): variable name that is being searched
        """

        if data.data in self._cache:
            var = self._cache[data.data]
            mission_control_utils.set_var(data.data, var)



    def pause_behaviour(self):
        """ Pauses the current subprocess """

        os.kill(self._process.pid, signal.SIGSTOP)
        self._paused = True


    def resume_behaviour(self):
        """ Resumes the current subprocess """

        os.kill(self._process.pid, signal.SIGCONT)
        self._paused = False

    def activate(self):
        """ Activates node """

        cmd = "exec python %s" % self._script
        self._process = subprocess.Popen(cmd,shell=True)
        self._running = True

    def is_thread_alive(self):
        """ Checks if the subprocess, in which the script runs, is alive 

        Returns:
            bool: True if subprocess is alive, False otherwise
        """

        self._process.poll()
        return self._process.returncode == None
