#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash
source /opt/open_mower_ros/version_info.env

# OSv2 debugging get controlled via env var DEBUG and has the ROSCONSOLE_CONFIG_FILE embedded
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
shopt -u nocasematch || true

exec -- "$@"
