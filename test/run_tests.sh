#!/bin/bash
rostest mission_control test_behaviour.launch --text
rostest mission_control test_behaviour_token_on_startup.launch --text
