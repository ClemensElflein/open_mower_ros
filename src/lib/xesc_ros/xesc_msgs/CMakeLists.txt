cmake_minimum_required(VERSION 2.8.3)
project(xesc_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES XescState.msg XescStateStamped.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
