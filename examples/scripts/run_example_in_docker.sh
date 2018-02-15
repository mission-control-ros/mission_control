#!/bin/bash
source /catkin_ws/devel/setup.bash
roscd mission_control/examples/scripts
./start_move_base_in_docker.sh

echo "Start use case..."
roslaunch mission_control use_case.launch
