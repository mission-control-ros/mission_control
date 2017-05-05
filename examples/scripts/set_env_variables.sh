#!/bin/bash
MISSION_CONTROL_PATH=$(pwd)
export TURTLEBOT_GAZEBO_MAP_FILE=/opt/ros/kinetic/share/turtlebot_navigation/maps/willow-2010-02-18-0.10.yaml
export TURTLEBOT_GAZEBO_WORLD_FILE=$MISSION_CONTROL_PATH/../worlds/willowgarage.world
export ROBOT_INITIAL_POSE='-x 27 -y -0.75'
