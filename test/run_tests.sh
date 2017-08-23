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
  show_test_coverage.launch
)

EXIT_CODE=0
FAILED=0
WHAT_FAILED=()

for TEST in ${TESTS[@]}; do

  CMD="rostest mission_control ${TEST}"
  eval $CMD

  TEST_RESULT=$?

  if [ $TEST_RESULT != 0 ]; then
    EXIT_CODE=1
    ((FAILED++))
    WHAT_FAILED+=($TEST)
  fi

done

rm -f $(rospack find mission_control)/test.log

echo
echo "Totally ${FAILED} test(s) failed!"
echo

if [ $FAILED != 0 ]; then
  echo "Launch files that failed:"
  for TEST in ${WHAT_FAILED[@]}; do
    echo $TEST
  done
fi

exit $EXIT_CODE
