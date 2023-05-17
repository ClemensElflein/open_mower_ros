# FTCLocalPlanner

This repository contains a very simple "follow the carrot" local planner implementation.

## Description
The planner follows a given path as exactly as possible. It calculate the position of the carrot by
taking robots velocity limits into account. If deviation between carrot and robot is above a given
threshold, the planner stops.

Also it checks for collisions of carrot with obstacles as well as collisions of robot footprint at actual robot pose. It doesn't implement obstacle aviodance but obstacle detection. Following situation will cause the 
planner to exit:
- goal reached
- goal (pose) timeout reached
- deviation between carrot and robot 
- collision of carrot with obstacles
- collision of robot footprint with obstacles

## Published Topics
- **`global_point`** (geometry_msgs/PoseStamped) pose of carrot
- **`global_plan`** (nav_msgs/Path) global path to follow
- **`debug_pid`** (ftc_local_planner/PID) debug information of PID calculation
- **`costmap_marker`** (visualization_msgs/Marker) debug information of costmap check

