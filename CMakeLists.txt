cmake_minimum_required(VERSION 3.0.2)
project(knowrob_designator)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  std_msgs
  message_generation
  knowrob_ros  # for KnowRobRosLib
)

add_action_files(
  DIRECTORY action
  FILES LogDesignator.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs rospy std_msgs message_runtime knowrob_ros
)
