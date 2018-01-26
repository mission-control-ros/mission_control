#!/bin/bash
./start_move_base_in_docker.sh

echo "Start use case..."
roslaunch mission_control use_case.launch
