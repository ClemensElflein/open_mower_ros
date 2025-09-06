# /etc/profile.d/openmower_profile.sh
# Ensure interactive shells in the container have ROS and the catkin workspace sourced.

# Source ROS and the workspace overlay (if present)
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    . "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if [ -f "/opt/open_mower_ros/devel/setup.bash" ]; then
    . "/opt/open_mower_ros/devel/setup.bash"
fi

if [ -n "${PS1:-}" ] && [ "${OM_IN_CONTAINER:-}" = "1" ]; then
    stack="${OM_STACK:-openmower}"
    export PS1="\[\e[1;32m\]\u@\[\e[1;34m\]${stack}\[\e[1;32m\]@\h\[\e[0m\]:\[\e[36m\]\w $\[\e[0m\] "
fi
