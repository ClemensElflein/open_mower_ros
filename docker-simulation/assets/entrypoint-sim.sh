#!/bin/bash
set -e

# GPU passthrough is opt-in (see docker-compose.yaml). If the host didn't map a render
# node in, fall back to software rendering so rviz still comes up everywhere.
if [ -d /dev/dri ] && ls /dev/dri/render* >/dev/null 2>&1; then
    echo "[entrypoint-sim] GPU render node found under /dev/dri - using hardware-accelerated rendering"
    export LIBGL_ALWAYS_SOFTWARE=0
else
    echo "[entrypoint-sim] No GPU render node found - falling back to software rendering (llvmpipe)"
    export LIBGL_ALWAYS_SOFTWARE=1
fi

exec /usr/bin/supervisord -n -c /etc/supervisor/conf.d/simulation.conf
