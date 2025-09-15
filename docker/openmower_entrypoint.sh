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
    # If OM_V2 is set (to anything), set HARDWARE_PLATFORM=2, else 1
    if [[ -n ${OM_V2+x} ]]; then
        export HARDWARE_PLATFORM=2
    else
        export HARDWARE_PLATFORM=1
    fi
    export OM_LEGACY_CONFIG_MODE=True
    export ESC_TYPE=$OM_MOWER_ESC_TYPE
    export MOWER=$OM_MOWER
fi

# If a legacy OS user pulls a non-legacy OS image, MOWER is not set.
if [[ -z "${MOWER:-}" ]]; then
    echo "ERROR: MOWER is not set." >&2
    echo "Hint: If you're running a legacy OpenMowerOS (dated before Sep 2025), change OM_VERSION to a version prefixed with 'releases-' or suffixed with '-legacy'." >&2
    exit 2
fi

# source the hardware specific default environment (default wheel ticks, antenna position etc)
source "$(rospack find open_mower)/params/hardware_specific/$MOWER/default_environment.sh"

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
