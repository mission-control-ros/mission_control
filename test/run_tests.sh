#!/bin/bash
rostest mission_control test_behaviour_token_on_startup.launch --text
rostest mission_control test_behaviour_token_passing.launch --text
rostest mission_control test_behaviour_token_passing2.launch --text
rostest mission_control test_behaviour_variable_passing.launch --text
rostest mission_control test_behaviour_subprocess_token_passing.launch --text
rostest mission_control test_behaviour_subprocess_token_passing_with_cpp_script.launch --text
rostest mission_control test_watchdog.launch --text
rostest mission_control test_watchdog_fail_safe.launch --text
rostest mission_control test_watchdog_fail_safe_subprocess.launch --text
rostest mission_control test_mission_control_utils_set_var.launch --text
rostest mission_control test_mission_control_utils_get_var.launch --text
rostest mission_control test_behaviour.launch --text
