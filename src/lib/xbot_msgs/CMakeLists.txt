cmake_minimum_required(VERSION 3.0.2)
project(xbot_msgs)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        message_generation)


add_message_files(
        FILES
        AbsolutePose.msg
        WheelTick.msg
        SensorInfo.msg
        SensorDataString.msg
        SensorDataDouble.msg
        RobotState.msg
        Map.msg
        MapArea.msg
        ActionInfo.msg
        ActionRequest.msg
        ActionResponse.msg
        MapOverlay.msg
        MapOverlayPolygon.msg
)

add_service_files(
        FILES
        RegisterActionsSrv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package()
