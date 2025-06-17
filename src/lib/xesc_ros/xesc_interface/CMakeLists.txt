cmake_minimum_required(VERSION 3.1)

project(xesc_interface)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)
catkin_package(
        INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(${catkin_DEFINITIONS})

install(DIRECTORY include ${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
