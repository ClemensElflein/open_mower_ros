#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash

if [[ "$1" == "osv1" ]]; then
    source /config/mower_config.sh
fi

# source the hardware specific default environment (default wheel ticks, antenna position etc)
source "$(rospack find open_mower)/params/hardware_specific/$OM_MOWER/default_environment.sh"

source /opt/open_mower_ros/version_info.env

# OSv2 debugging get controlled via env var DEBUG and has the ROSCONSOLE_CONFIG_FILE embedded
shopt -s nocasematch
if [[ "$1" == "osv2" ]]; then
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
fi
shopt -u nocasematch || true

exec -- "$@"
