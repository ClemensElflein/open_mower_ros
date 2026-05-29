cmake_minimum_required(VERSION 3.14)
project(mower_analytics)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mower_msgs
)

include(FetchContent)

FetchContent_Declare(sentry-native
  GIT_REPOSITORY https://github.com/getsentry/sentry-native.git
  GIT_TAG        0.14.0
  GIT_SUBMODULES "external/crashpad"
)
set(SENTRY_BACKEND   "crashpad" CACHE STRING "" FORCE)
set(SENTRY_TRANSPORT "curl"     CACHE STRING "" FORCE)
FetchContent_MakeAvailable(sentry-native)

add_library(mower_analytics_sentry src/sentry_guard.cpp)
target_include_directories(mower_analytics_sentry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mower_analytics_sentry PUBLIC sentry::sentry ${catkin_LIBRARIES})
set_target_properties(mower_analytics_sentry PROPERTIES CXX_STANDARD 17)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mower_analytics_sentry
  CATKIN_DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

# crashpad_handler must be findable at runtime — place it in this package's bin dir
# so SentryGuard's sibling-path lookup resolves it for all other nodes too.
add_custom_command(TARGET mower_analytics_sentry POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CATKIN_DEVEL_PREFIX}/lib/mower_analytics
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:crashpad_handler>
    ${CATKIN_DEVEL_PREFIX}/lib/mower_analytics/crashpad_handler
  COMMENT "Copying crashpad_handler to devel/lib/mower_analytics/"
)
