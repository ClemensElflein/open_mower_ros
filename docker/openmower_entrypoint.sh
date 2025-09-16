#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash

# First arg should be mode now (osv1 or osv2)
MODE="$1"
if [[ -n "$MODE" ]]; then
    shift
fi

if [[ "$MODE" == "osv1" ]]; then
    source /config/mower_config.sh
    # If OM_V2 is truthy, set HARDWARE_PLATFORM=2 and new (yaml-based) config, else 1 and environment config
    if [[ "${OM_V2,,}" =~ ^(true|1|yes)$ ]]; then
        export HARDWARE_PLATFORM=2
    else
        export HARDWARE_PLATFORM=1
        export OM_LEGACY_CONFIG_MODE=True
    fi
    export ESC_TYPE=$OM_MOWER_ESC_TYPE
    export MOWER=$OM_MOWER
    # source the hardware specific default environment (default wheel ticks, antenna position etc)
    source "$(rospack find open_mower)/params/hardware_specific/$MOWER/default_environment.sh"

    # Set the recording path to $HOME for OSv1 to match the old behavior
    export RECORDINGS_PATH=$HOME
    export PARAMS_PATH=$HOME
fi

source /opt/open_mower_ros/version_info.env

# OSv2 debugging get controlled via env var DEBUG and has the ROSCONSOLE_CONFIG_FILE embedded
shopt -s nocasematch
if [[ "$MODE" == "osv2" ]]; then
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
