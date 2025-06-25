cmake_minimum_required(VERSION 3.0.2)
project(ftc_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            roscpp
		tf
		tf2
		tf2_geometry_msgs
		tf2_ros
		geometry_msgs
		std_msgs
		message_generation
        )


find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
		${Eigen3_INCLUDE_DIRS}
    )
add_definitions(${Eigen3_DEFINITIONS})


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/FTCPlanner.cfg
)

## Generate services in the 'srv' folder
add_service_files(
		FILES
		PlannerGetProgress.srv
)

add_message_files(
        FILES
        PID.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ftc_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
		tf
		tf2_geometry_msgs
		tf2_ros
)

catkin_package(
		CATKIN_DEPENDS message_runtime
)

add_library(ftc_local_planner 
	    src/ftc_planner.cpp
        src/oscillation_detector.cpp)
target_link_libraries(ftc_local_planner ${catkin_LIBRARIES})
add_dependencies(ftc_local_planner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ftc_local_planner ftc_local_planner_gencfg)
add_dependencies(ftc_local_planner ftc_local_planner_generate_messages_cpp)

add_library(backward_forward_recovery src/backward_forward_recovery.cpp)
target_link_libraries(backward_forward_recovery ${catkin_LIBRARIES})

install(TARGETS ftc_local_planner backward_forward_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )
       
install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
 
