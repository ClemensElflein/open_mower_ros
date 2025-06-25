cmake_minimum_required(VERSION 3.0.2)
project(xbot_positioning)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        xbot_msgs
        roscpp
        tf2_geometry_msgs
        tf2
        tf2_ros
        message_generation
        geometry_msgs
        )


find_package(
        Boost
)




add_message_files(
        FILES
        KalmanState.msg
)

add_service_files(
        FILES
        GPSControlSrv.srv
        SetPoseSrv.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)
catkin_package(
        #          INCLUDE_DIRS include
        #  LIBRARIES mower_comms
        #  CATKIN_DEPENDS mower_msgs roscpp serial
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(xbot_positioning
        src/xbot_positioning_core.h
        src/xbot_positioning_core.cpp
        src/xbot_positioning.cpp
        src/OrientationMeasurementModel.hpp
        src/OrientationMeasurementModel2.hpp
        src/PositionMeasurementModel.hpp
        src/SystemModel.hpp)

add_dependencies(xbot_positioning ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xbot_positioning ${catkin_LIBRARIES})
