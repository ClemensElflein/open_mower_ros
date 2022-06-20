#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/openmower/ROS/devel/setup.bash

exec "$@"
