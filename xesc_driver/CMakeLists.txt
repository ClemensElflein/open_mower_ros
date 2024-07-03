cmake_minimum_required(VERSION 2.8.3)
project(xesc_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  std_msgs
  serial
        xesc_msgs
        vesc_driver
        xesc_2040_driver
        xesc_yfr4_driver
        xesc_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xesc_driver
  CATKIN_DEPENDS pluginlib roscpp std_msgs xesc_msgs serial xesc_2040_driver vesc_driver xesc_yfr4_driver
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
add_library(xesc_driver
        src/xesc_driver.cpp
        include/xesc_driver/xesc_driver.h)

add_dependencies(xesc_driver ${catkin_EXPORTED_TARGETS})

# node executable
add_executable(xesc_driver_node src/xesc_driver_node.cpp)
add_dependencies(xesc_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(xesc_driver_node
        xesc_driver
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

install(TARGETS xesc_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

