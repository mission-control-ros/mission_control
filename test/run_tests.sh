#!/bin/bash

rm -f ~/.ros/.coverage

TESTS=(
  test_behaviour_token_on_startup.launch
  test_behaviour_token_passing.launch
  test_behaviour_token_passing2.launch
  test_behaviour_variable_passing.launch
  test_behaviour_token_passing_with_cpp_script.launch
  test_watchdog.launch
  test_watchdog_fail_safe.launch
  test_mission_control_utils_set_var.launch
  test_mission_control_utils_get_var.launch
  test_mission_control_utils_cpp_set_var.launch
  test_mission_control_utils_cpp_get_var.launch
  test_behaviour.launch
)

EXIT_CODE=0

for TEST in ${TESTS[@]}; do

  CMD="rostest mission_control ${TEST}"
  eval $CMD

  TEST_RESULT=$?

  if [ $TEST_RESULT != 0 ]; then
    EXIT_CODE=1
  fi

done

exit $EXIT_CODE
