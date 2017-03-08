# Mission control for autonomous robots

This project is for creating mission control program for autonomous robots, by using either SMACH's Statemachines or custom scripts. SMACH can be found at http://wiki.ros.org/smach.

Mission control program consists of 1 to N nodes. Each node has a custom script or a state machine, which is executed when the node becomes active and has aqcuired token. 

Every node has a priority level from 1 to N. Multiple nodes may have same priority levels (NB! Same priority level nodes execute asynchronously). Lower priority number shows higher priority.

The token shows which priority level is allowed to execute its state machine or custom script. When the node has the token other nodes may request the token for themselves. The token is released only when requester node's priority level is higher than active node's priority level or node has finished executing.

## Prerequisites

Mission control system will only work with ROS (Robot Operating System). Tutorial, on how to install ROS, can be found at http://wiki.ros.org/ROS/Installation.

## Installation

```
cd ~/catkin_ws/src
git clone https://github.com/mission-control-ros/mission_control.git
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
```

## Creating custom scripts

In the beginning of the custom script function ros_init from module mission_control_utils must be called. This is required to receive and send variables to other scripts. 
When the script doesn't need any external variables and doesn't share its own variables, then function call ros_init is not required.

### Variable management inside custom scripts

To access external variables initialized in other scripts use function get_var() from module mission_control_utils. Function's first parameter is the variable's name that is being requested. Second parameter is the default value that is returned, if requested variable is not found. Default value is optional and if it is not supplied function will return None if requested variable is not found.

To make your script's variables available for other scripts use function set_var() from module mission_control_utils. Function's first parameter is variable's name and second parameter is for variable's value.

## Deployment for custom scripts

#### ROS launch file's node example
```
  <node name="node1" pkg="mission_control" type="behaviour_subprocess_node.py" output="screen">
    <param name="priority" value="3" />
    <param name="token" value="1" />
    <param name="active" value="int(self.get_var('counter6', 10)) &lt;= 10 and int(self.get_var('counter6', 10)) != 0" />
    <param name="script" value="$(find mission_control)/examples/scripts/custom_script_priority6.py" />
  </node>
```
#### Node's parameters explanation

* node type - script behaviour_subprocess_node.py has to be used
* priority - defines nodes priority. Smaller number shows higher priority. Values are in range 1..N
* token - shows which node has token on startup
* active - string which will be evaluated to boolean value. It shows under which conditions node is allowed to be active. To access variables that initialized in scripts use function self.get_var('your_variables_name'). Function's self.get_var first parameter is the variable's name that is being requested. Second optional parameter is default value that is returned if no variable with given name is found. When no default value is supplied None is returned when requested variable is not found.
* script - full path to script which will be executed when node becomes active

