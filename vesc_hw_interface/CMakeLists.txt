cmake_minimum_required(VERSION 2.8.3)
project(vesc_hw_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  hardware_interface
  joint_limits_interface
  controller_manager
  pluginlib
  urdf
  vesc_driver
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    serial
    hardware_interface
    joint_limits_interface
    controller_manager
    pluginlib
    urdf
    vesc_driver
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/vesc_hw_interface.cpp
  src/vesc_servo_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
  src/vesc_hw_interface_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
