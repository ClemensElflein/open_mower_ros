# /etc/profile.d/openmower_profile.sh
# Ensure interactive shells in the container have ROS and the catkin workspace sourced.

# Source ROS and the workspace overlay (if present)
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    . "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if [ -f "/opt/open_mower_ros/devel/setup.bash" ]; then
    . "/opt/open_mower_ros/devel/setup.bash"
fi
