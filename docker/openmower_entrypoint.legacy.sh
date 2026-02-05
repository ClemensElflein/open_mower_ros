#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash

# setup om environment
source /opt/open_mower_ros/version_info.env
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

# Ensure stdout and stderr are unbuffered to get logging in real time order
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
export PYTHONUNBUFFERED=1

exec -- "$@"
