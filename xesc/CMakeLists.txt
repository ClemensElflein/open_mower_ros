cmake_minimum_required(VERSION 2.8.3)
project(xesc)
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
        CATKIN_DEPENDS xesc_msgs
)
catkin_package(
        INCLUDE_DIRS include
)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
