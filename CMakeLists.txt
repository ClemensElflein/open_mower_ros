cmake_minimum_required(VERSION 3.0.2)
project(openbot_msgs)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        message_generation)


add_message_files(
        FILES
        AbsolutePoseInfo.msg
        WheelTick.msg
)

#add_service_files(
#        FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package()
