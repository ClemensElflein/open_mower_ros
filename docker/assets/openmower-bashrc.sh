# This get extend user's ~/.bashrc

# Interactive shells PS1 prefix opt-in via STACK_SHELL=1 to avoid non-interactive side effects.
if [ -n "${PS1:-}" ] && [ "${STACK_SHELL:-}" = "1" ]; then
    stack="${STACK_NAME:-openmower}"
    export PS1="\[\e[1;34m\][${stack}]\[\e[0m\] $PS1"
fi

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
