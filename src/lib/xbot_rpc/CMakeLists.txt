cmake_minimum_required(VERSION 3.0.2)
project(xbot_rpc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation)

add_message_files(
        FILES
        RpcRequest.msg
        RpcResponse.msg
        RpcError.msg
)

add_service_files(
        FILES
        RegisterMethodsSrv.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

add_subdirectory(../json json EXCLUDE_FROM_ALL)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(xbot_rpc
  src/provider.cpp
)

target_link_libraries(xbot_rpc nlohmann_json::nlohmann_json)

add_dependencies(xbot_rpc
  ${PROJECT_NAME}_gencpp   # <-- depends on generated C++ message headers
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS xbot_rpc
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xbot_rpc
  CATKIN_DEPENDS std_msgs message_runtime
)
