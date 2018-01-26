#!/bin/bash
./start_move_base.sh

echo "Start use case..."
roslaunch mission_control use_case.launch
