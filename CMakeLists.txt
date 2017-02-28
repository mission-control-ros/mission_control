cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   Variable.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )
