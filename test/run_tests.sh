#!/bin/bash
rostest mission_control test_behaviour.launch
rostest mission_control test_behaviour_token_on_startup.launch
rostest mission_control test_behaviour_token_passing.launch
rostest mission_control test_behaviour_variable_passing.launch
rostest mission_control test_behaviour_subprocess_token_passing.launch
