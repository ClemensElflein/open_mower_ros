# This get extend user's ~/.bashrc

# Source ROS and the workspace overlay (if present)
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    . "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if [ -f "/opt/open_mower_ros/devel/setup.bash" ]; then
    . "/opt/open_mower_ros/devel/setup.bash"
fi

# Also expose version info if present
if [ -f "/opt/open_mower_ros/version_info.env" ]; then
    . "/opt/open_mower_ros/version_info.env"
fi
