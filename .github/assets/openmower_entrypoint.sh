#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash
source /config/mower_config.sh

# source the hardware specific default environment (default wheel ticks, antenna position etc)
source $(rospack find open_mower)/params/hardware_specific/$OM_MOWER/default_environment.sh

exec "$@"
