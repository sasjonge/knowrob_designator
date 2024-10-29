cmake_minimum_required(VERSION 3.0.2)
project(knowrob_designator)

# Add this line at the top to find messages and services
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation  # Add this for custom services
)

# Add this section to enable service generation
add_service_files(
  FILES
  SimpleQuery.srv
)

# Add this section to generate messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Ensure catkin_package includes message runtime dependencies
catkin_package(
  CATKIN_DEPENDS message_runtime
)
