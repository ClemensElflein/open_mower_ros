# Set default GPS antenna offset
export OM_ANTENNA_OFFSET_X=${OM_ANTENNA_OFFSET_X:-0.1}
export OM_ANTENNA_OFFSET_Y=${OM_ANTENNA_OFFSET_Y:-0.0}

# Set distance between wheels in m
export OM_WHEEL_DISTANCE_M=${WHEEL_DISTANCE_M:-0.325}

# Set default ticks/m
export OM_WHEEL_TICKS_PER_M=${OM_WHEEL_TICKS_PER_M:-1964.0}

# Serial ports change when kernel version >= 6.1.28
# https://forums.raspberrypi.com/viewtopic.php?t=347868#p2106791
KERNEL_VERSION_MAJOR=$(uname -r | cut -d. -f1)
KERNEL_VERSION_MINOR=$(uname -r | cut -d. -f2)
KERNEL_VERSION_PATCH=$(uname -r | cut -d. -f3 | sed -e 's/^\([0-9]\+\).*/\1/')
KERNEL_VERSION=$((KERNEL_VERSION_MAJOR * 10000 + KERNEL_VERSION_MINOR * 100 + KERNEL_VERSION_PATCH))
if [ $KERNEL_VERSION -lt 60128 ]; then
    export OM_XESC_LEFT_PORT=${OM_XESC_LEFT_PORT:-/dev/ttyAMA4}
    export OM_XESC_RIGHT_PORT=${OM_XESC_RIGHT_PORT:-/dev/ttyAMA2}
    export OM_XESC_MOWER_PORT=${OM_XESC_MOWER_PORT:-/dev/ttyAMA3}
    export OM_GPS_PORT=${OM_GPS_PORT:-/dev/ttyAMA1}
fi
