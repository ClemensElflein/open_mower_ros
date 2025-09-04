#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash
source /config/mower_config.sh

# source the hardware specific default environment (default wheel ticks, antenna position etc)
source "$(rospack find open_mower)/params/hardware_specific/$OM_MOWER/default_environment.sh"

source /opt/open_mower_ros/version_info.env

# Enable case-insensitive matching for the case statement
shopt -s nocasematch

case "${DEBUG:-0}" in
    1|true|yes|on|y)
        export ROSOUT_DISABLE_FILE_LOGGING=False
        unset ROSCONSOLE_CONFIG_FILE
    ;;
    *)
        export ROSCONSOLE_CONFIG_FILE=/config/rosconsole.config
        export ROSOUT_DISABLE_FILE_LOGGING=True
    ;;
esac

exec -- "$@"
