cmake_minimum_required(VERSION 2.8.3)
project(xesc_2040_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  std_msgs
  serial
        vesc_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xesc_2040_driver
  CATKIN_DEPENDS nodelet pluginlib roscpp std_msgs vesc_msgs serial
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
add_library(xesc_2040_driver
        src/xesc_2040_interface.cpp
        include/xesc_2040_driver/COBS.h
        include/xesc_2040_driver/xesc_2040_datatypes.h)

add_dependencies(xesc_2040_driver ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS xesc_2040_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# node executable
add_executable(xesc_2040_driver_node src/xesc_2040_driver_node.cpp)
add_dependencies(xesc_2040_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(xesc_2040_driver_node
        xesc_2040_driver
        ${catkin_LIBRARIES}
        )


#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
