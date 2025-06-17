cmake_minimum_required(VERSION 2.8.3)
project(xesc_yfr4_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  std_msgs
  serial
        xesc_msgs
        xesc_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xesc_yfr4_driver
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
add_library(xesc_yfr4_driver
        src/xesc_yfr4_interface.cpp
        src/xesc_yfr4_driver.cpp
        include/xesc_yfr4_driver/COBS.h
        include/xesc_yfr4_driver/xesc_yfr4_datatypes.h include/xesc_yfr4_driver/xesc_yfr4_driver.h)

add_dependencies(xesc_yfr4_driver ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS xesc_yfr4_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
