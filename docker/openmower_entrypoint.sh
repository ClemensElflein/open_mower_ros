#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash
source /opt/open_mower_ros/version_info.env

# Ensure stdout and stderr are unbuffered to get logging in real time order
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
export PYTHONUNBUFFERED=1

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

exec stdbuf -oL -eL -- "$@"
