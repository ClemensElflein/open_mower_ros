cmake_minimum_required(VERSION 3.11)
include(FetchContent)
project(xbot_remote)

## Compile as C++17
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
        xbot_msgs
        roscpp
        geometry_msgs
        std_msgs
        )


find_package(
        Boost
)

find_package(websocketpp REQUIRED)


catkin_package(

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

add_executable(xbot_remote
        src/xbot_remote.cpp)


add_dependencies(xbot_remote ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xbot_remote ${catkin_LIBRARIES} nlohmann_json::nlohmann_json websocketpp::websocketpp)
