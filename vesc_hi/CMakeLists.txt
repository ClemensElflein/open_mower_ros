cmake_minimum_required(VERSION 2.8.3)
project(vesc_hi)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  hardware_interface
  controller_manager
  vesc_driver
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vesc_hi
  CATKIN_DEPENDS roscpp serial hardware_interface controller_manager vesc_driver
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(vesc_hi
  src/vesc_hi.cpp
  src/vesc_servo_controller.cpp
)
add_dependencies(vesc_hi ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_hi ${catkin_LIBRARIES})

add_executable(vesc_hi_node
  src/vesc_hi_node.cpp
)
add_dependencies(vesc_hi_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_hi_node ${catkin_LIBRARIES} vesc_hi)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
