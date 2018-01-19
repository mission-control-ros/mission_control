#!/bin/bash
source /catkin_ws/devel/setup.bash
roscd mission_control/examples/scripts
source set_env_variables.sh

echo "Start xvfb..."
Xvfb -shmem -screen 0 1280x1024x24 &
sleep 2

echo "Start gazebo server..."
DISPLAY=:0 roslaunch mission_control use_case_world.launch gui:=false &
echo "Wait for 2 minutes, because some models need to be downloaded..."
sleep 120

echo "Spawn model..."
roslaunch mission_control use_case_robot.launch &
echo "Wait for 5 seconds..."
sleep 5

echo "Start amcl..."
roslaunch mission_control amcl.launch &
echo "Wait for 10 seconds, to get the message 'odom received!'..."
sleep 10

echo "Start use case..."
roslaunch mission_control use_case.launch
