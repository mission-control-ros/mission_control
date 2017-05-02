cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  smach
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Variable.msg
  Answer.msg
  Health.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES mission_control_utils
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  mission_control_utils src/mission_control_utils.cpp
)

target_link_libraries(mission_control_utils ${catkin_LIBRARIES})

add_executable(custom_script_priority6 examples/scripts/custom_script_priority6.cpp src/mission_control_utils.cpp)

add_dependencies(custom_script_priority6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(custom_script_priority6
  ${catkin_LIBRARIES}
)

add_executable(test_custom_script_priority6 test/test_custom_script_priority6.cpp src/mission_control_utils.cpp)

add_dependencies(test_custom_script_priority6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_custom_script_priority6
  ${catkin_LIBRARIES}
)
