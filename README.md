# Mission control for autonomous robots 

[![Build Status](https://travis-ci.org/mission-control-ros/mission_control.svg?branch=master)](https://travis-ci.org/mission-control-ros/mission_control)

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

To make your script's variables available for other scripts use function set_var() from module mission_control_utils. Function's first parameter is variable's name and second parameter is for variable's value. Third parameter is time to live which shows how long the given variable is held in requested object's cache. Third parameter is optional and if it's not given, variable is held in cache as long as the object is alive.

## Deployment for custom scripts

#### ROS launch file's node example for python scripts
```
  <node name="node1" pkg="mission_control" type="behaviour_node.py" output="screen">
    <param name="priority" value="3" />
    <param name="active" value="int(self.get_var('counter6', 10)) &lt;= 10 and int(self.get_var('counter6', 10)) != 0" />
    <param name="script" value="$(find mission_control)/examples/scripts/custom_script_priority6.py" />
    <param name="wait_before_startup" value="2" />
    <param name="debug" value="1" />
  </node>
```

#### ROS launch file's node example for C++ executables
```
  <node name="node1" pkg="mission_control" type="behaviour_node.py" output="screen">
    <param name="priority" value="3" />
    <param name="active" value="int(self.get_var('counter6', 10)) &lt;= 10 and int(self.get_var('counter6', 10)) != 0" />
    <param name="script" value="mission_control custom_script_priority6" />
    <param name="wait_before_startup" value="2" />
    <param name="debug" value="1" />
  </node>
```

#### Node's parameters explanation

* node type - Script behaviour_subprocess_node.py has to be used
* priority - Defines nodes priority. Smaller number shows higher priority. Values are in range 1..N
* active - String which will be evaluated to boolean value. It shows under which conditions node is allowed to be active. To access variables that initialized in scripts use function self.get_var('your_variables_name'). Function's self.get_var first parameter is the variable's name that is being requested. Second optional parameter is default value that is returned if no variable with given name is found. When no default value is supplied None is returned when requested variable is not found.
* script - For python scripts full path to script which will be executed when node becomes active. For C++ scripts string containing ROS package name and C++ executable separated with space.
* wait_before_startup (optional) - How many seconds node sleeps before execution in order to subscribe to all the topics. Default value is 1 second.
* debug (optional) - Debug level which determines how many debug messages are displayed. This value ranges from 0 to 3. Zero turns debugging off. Default value is 0.

## Creating SMACH StateMachine scripts

In order for node to access SMACH StateMachine object, a variable which contains StateMachine object must be declared on a global level in the script. 
In the end of the file user has to check if the file is being ran or just imported. Inside that if clause user has to to ros_init and execute the statemachine. For example:
```
sm = smach.StateMachine(outcomes=['outcome4'])

with sm:
    smach.StateMachine.add('FOO', Foo(),
                           transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
    smach.StateMachine.add('BAR', Bar(),
                           transitions={'outcome1':'FOO'})

if __name__ == '__main__':
    mission_control_utils.ros_init("statemachine")
    sm.execute()
```

This is necessary, because when the smach.Statemachine object is not being executed, the node imports the python script and tries to search for variables, that are asked from it.

### Variable management inside SMACH StateMachine object

To access external variables initialized in StateMachine object use function get_var() from module mission_control_utils. Function's first parameter is the variable's name that is being requested. Second parameter is the default value that is returned, if requested variable is not found. Default value is optional and if it is not supplied function will return None if requested variable is not found.

To make your StateMachine's variables available for other Statemachines use function set_var() from module mission_control_utils. Function's first parameter is variable's name and second parameter is for variable's value. Third parameter is time to live which shows how long the given variable is held in requested object's cache. Third parameter is optional and if it's not given, variable is held in cache as long as the object is alive.

## Deployment for StateMachine objects
```
  <node name="node1" pkg="mission_control" type="behaviour_node.py" output="screen">
    <param name="priority" value="6" />
    <param name="active" value="int(self.get_var('counter6')) &lt;= 10 and bool(self.get_var('fuel_tank')) and int(self.get_var('counter6')) != 0" />
    <param name="script" value="$(find mission_control)/examples/scripts/state_machine_priority6.py" />
    <param name="wait_before_startup" value="2" />
    <param name="debug" value="1" />
  </node>
```

#### Node's parameters explanation

* node type - script behaviour_node.py has to be used
* priority - defines nodes priority. Smaller number shows higher priority. Values are in range 1..N
* active - string which will be evaluated to boolean value. It shows under which conditions node is allowed to be active. To access variables that initialized in scripts use function self.get_var('your_variables_name'). Function's self.get_var first parameter is the variable's name that is being requested. Second optional parameter is default value that is returned if no variable with given name is found. When no default value is supplied None is returned when requested variable is not found.
* script - full path to script which holds the SMACH StateMachine that will be executed
* wait_before_startup (optional) - how many seconds node sleeps before execution in order to subscribe to all the topics. Default value is 1 second.
* debug (optional) - Debug level which determines how many debug messages are displayed. This value ranges from 0 to 3. Zero turns debugging off. Default value is 0.

## Watchdog

Monitors all the active nodes and tells the user when some of the nodes become unresponsive.

## Deployment for watchdog

#### ROS launch file's node example
```
  <node name="watchdog" pkg="mission_control" type="watchdog_node.py" output="screen">
    <param name="debug" type="bool" value="true" />
    <param name="debug_file" value="$(find mission_control)/test.log" />
    <param name="node_dead_after" value="0.5" />
  </node>
```

#### Node's parameters explanation
* node type - script watchdog_node has to be used
* debug (optional) - if true debug info is displayed, default value is false
* debug_file (optional) - logs all the debug info into given file
* node_dead_after (optional) - in seconds the amount of time passed for node to be declared unresponsive, default value is 2 seconds

## Watchdog fail safe

Starts after watchdog has detected that some node has unexpectedly died or won't send health messages. Both SMACH StateMachine objects and custom scripts can be used.

## Deployment for watchdog fail safe

#### ROS launch file's node example for SMACH StateMachines
```
  <node name="fail_safe" pkg="mission_control" type="behaviour_fail_safe_node.py" output="screen">
    <param name="script" value="$(find mission_control)/examples/scripts/state_machine_fail_safe.py" />
    <param name="debug" value="1" />
  </node>
```

#### Node's parameters explanation
* node type - script behaviour_fail_safe_node.py has to be used
* state_machine - full path to script which holds the SMACH StateMachine that will be executed
* debug (optional) - Debug level which determines how many debug messages are displayed. This value ranges from 0 to 3. Zero turns debugging off. Default value is 0.


#### ROS launch file's node example for custom scripts
```
  <node name="fail_safe" pkg="mission_control" type="behaviour_fail_safe_node.py" output="screen">
    <param name="script" value="$(find mission_control)/examples/scripts/custom_script_fail_safe.py" />
  </node>
```

#### Node's parameters explanation
* node type - script behaviour_subprocess_fail_safe_node.py has to be used
* script - For python scripts full path to script which will be executed when node becomes active. For C++ scripts string containing ROS package name and C++ executable separated with space.
* debug (optional) - Debug level which determines how many debug messages are displayed. This value ranges from 0 to 3. Zero turns debugging off. Default value is 0.


## Use case

To see how mission control works in simulation using Gazebo, follow these steps.

Firstly be sure that you have installed ROS Desktop-Full installation that can be found here: http://wiki.ros.org/kinetic/Installation/Ubuntu

Secondly install ros-kinetic-turtlebot-navigation and ros-kinetic-turtlebot-gazebo
```
sudo apt-get install ros-kinetic-turtlebot-navigation
sudo apt-get install ros-kinetic-turtlebot-gazebo
```

If that is done, run these commands (NB! if the command roslaunch mission_control use_case_world.launch gives an error with exit code 139, then try to run it again until it starts up. There seems to be an error with the link between ROS and Gazebo)
```
roscd mission_control/examples/scripts/
source set_env_variables.sh
roslaunch mission_control use_case_world.launch
```

When gazebo has started and you can see the willow garage office, then open a NEW terminal and run these commands, which will spawn a robot into the gazebo world
```
roscd mission_control/examples/scripts/
source set_env_variables.sh
roslaunch mission_control use_case_robot.launch
```

If you can see the robot in the willow garage office, the again open a NEW terminal and run these commands
```
roscd mission_control/examples/scripts/
source set_env_variables.sh
roslaunch mission_control amcl.launch
```

When the previous command prints out message 'odom received!', then once more open a NEW terminal and run this command
```
roslaunch mission_control use_case.launch
```

This command should emulate robots battery and also give commands to move between two rooms. When battery level drops below 20, then the robot should move to a thrid room, where it's battery should recharge. After the battery level is above 95, it should start moving between two rooms again.
