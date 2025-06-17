cmake_minimum_required(VERSION 3.11)
include(FetchContent)
project(xbot_monitoring)

## Compile as C++17
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
        xbot_msgs
        roscpp
        geometry_msgs
        std_msgs
        grid_map_core
        grid_map_ros
        grid_map_msgs
        )


find_package(
        Boost
)


## System dependencies are found with CMake's conventions
find_package(PahoMqttCpp REQUIRED)
set(PahoMqttCpp_LIBRARIES PahoMqttCpp::paho-mqttpp3)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES mower_comms
        #  CATKIN_DEPENDS mower_msgs roscpp serial
        #  DEPENDS system_lib
        DEPENDS PahoMqttCpp

)

# Fetch JSON
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

###########
## Build ##
###########

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(xbot_monitoring
        src/xbot_monitoring.cpp)


add_dependencies(xbot_monitoring ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xbot_monitoring ${catkin_LIBRARIES} ${PahoMqttCpp_LIBRARIES} nlohmann_json::nlohmann_json)

add_executable(xbot_sensor_example
        src/xbot_sensor_example.cpp)


add_dependencies(xbot_sensor_example ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xbot_sensor_example ${catkin_LIBRARIES})
add_executable(heatmap_generator
        src/heatmap_generator.cpp)


add_dependencies(heatmap_generator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(heatmap_generator ${catkin_LIBRARIES})
