cmake_minimum_required(VERSION 2.8.3)
project(vesc_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  std_msgs
  xesc_msgs
  serial
        xesc_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vesc_driver
  CATKIN_DEPENDS pluginlib roscpp std_msgs xesc_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# driver libraries
add_library(vesc_driver
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)

add_dependencies(vesc_driver ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS vesc_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
