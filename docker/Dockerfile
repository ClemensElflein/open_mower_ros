# syntax=docker/dockerfile:1.7-labs
# Get a base/common image for OSv1 & OSv2
FROM docker.io/ros:noetic-ros-base-focal AS base
ENV DEBIAN_FRONTEND=noninteractive

# Install required base packages only (no nginx/mosquitto here). Use BuildKit cache for apt.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install --yes --no-install-recommends \
    git \
    python3.8-venv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep (network call). Cache apt metadata as well.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rosdep update --rosdistro $ROS_DISTRO

####################################################################################################

# Get slic3r_coverage_planner and build that. We will pull the finished install folder from this.
# This stage should cache most of the time, that's why it's not derived from the fetch stage, but copies stuff instead.
FROM base AS slic3r

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r planner from the repo (this will cache if unchanged)
COPY --link src/lib/slic3r_coverage_planner /opt/slic3r_coverage_planner_workspace/src

WORKDIR /opt/slic3r_coverage_planner_workspace
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked bash -euxo pipefail <<'EOF'
rosdep install --from-paths src --ignore-src --simulate \
    | sed '1d' \
    | sort \
    | tr -d '\n' \
    | sed 's/  apt-get install//g' > apt-install_list
apt-get update
xargs -r -a apt-install_list apt-get install --no-install-recommends -y || true
apt-get install --no-install-recommends -y libasio-dev
rm -rf /var/lib/apt/lists/* apt-install_list
EOF
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /opt/slic3r_coverage_planner_workspace/devel/setup.bash && catkin_make -DCMAKE_INSTALL_PREFIX=/opt/prebuilt/slic3r_coverage_planner install"

####################################################################################################

# Fetch the repo and assemble the list of dependencies. We will pull these in the next step and actually run install on them
# If the package list is the same as last time, the apt install step is cached as well which saves a lot of time.
# Since the list gets sorted, it will be the same each time and the cache will know that by file checksum in the COPY command.
# We can't use this stage as base for the next, because this stage is run every time and would therefore invalidate the cache of the next stage.
FROM base AS dependencies

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /opt/open_mower_ros

# This creates the sorted list of apt-get install commands.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=src,target=src bash -euxo pipefail <<'EOF'
apt-get update
rosdep install --from-paths src --ignore-src --simulate \
  | sed '1d' \
  | sort \
  | tr -d '\n' \
  | sed 's/  apt-get install//g' > /apt-install_list
rm -rf /var/lib/apt/lists/*
EOF

####################################################################################################
# We can't derive this from "dependencies" because "dependencies" will be rebuilt every time, but apt install should only be done if needed
# Assemble final OSv1 image and add nginx/mosquitto
FROM base AS assemble-osv1

ENV DEBIAN_FRONTEND=noninteractive

#Fetch the slic3r built earlier, this only changes if slic3r was changed (probably never)
COPY --link --from=slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner

#Fetch the list of packages, this only changes if new dependencies have been added (only sometimes)
COPY --link --from=dependencies /apt-install_list /apt-install_list
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) libasio-dev mosquitto nginx && \
    rm -rf /var/www /etc/nginx/sites-enabled/* && \
    rm -rf /var/lib/apt/lists/*

# Legacy service configs
COPY --link ./docker/assets/mosquitto.conf /etc/mosquitto/mosquitto.conf
COPY --link ./docker/assets/nginx.conf /etc/nginx/conf.d/default.conf

COPY --link ./ /opt/open_mower_ros

#delete prebuilt libs (so that they won't shadow the preinstalled ones)
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

WORKDIR /opt/open_mower_ros

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /opt/open_mower_ros/src && catkin_init_workspace && cd .. && source /opt/prebuilt/slic3r_coverage_planner/setup.bash && catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner"

RUN echo "export OM_SOFTWARE_VERSION=$(git describe --tags || git describe --always || echo "v0.0.0")" > version_info.env

COPY ./docker/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

ENTRYPOINT ["/openmower_entrypoint.sh", "osv1"]
CMD ["bash", "-c", "service nginx start; service mosquitto start; roslaunch open_mower open_mower.launch --screen"]


####################################################################################################
# We can't derive this from "dependencies" because "dependencies" will be rebuilt every time, but apt install should only be done if needed
# Assemble final OSv2 image (without nginx/mosquitto)
FROM base AS assemble-osv2

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r built earlier
COPY --link --from=slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner

# Fetch the list of packages
COPY --link --from=dependencies /apt-install_list /apt-install_list
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) libasio-dev && \
    rm -rf /var/lib/apt/lists/*

COPY --link ./ /opt/open_mower_ros

# delete prebuilt libs (so that they won't shadow the preinstalled ones)
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

WORKDIR /opt/open_mower_ros

# Single install build (skip separate devel build). Creates devel + install, we later discard sources.
RUN bash -exo pipefail <<'EOF'
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/prebuilt/slic3r_coverage_planner/setup.bash
cd /opt/open_mower_ros/src && catkin_init_workspace || true
cd /opt/open_mower_ros
catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner -DCMAKE_INSTALL_PREFIX=/opt/open_mower_install install
EOF

# Build-time sanity check: ensure key packages resolvable from install space
RUN bash -exo pipefail <<'EOF'
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/open_mower_install/setup.bash || true
rospack find open_mower > /dev/null 2>&1 || { echo 'ERROR: open_mower package not found in install space'; exit 1; }
EOF

# Capture version info into install dir before pruning workspace
RUN bash -c 'echo "export OM_SOFTWARE_VERSION=$(git -C /opt/open_mower_ros describe --tags || git -C /opt/open_mower_ros describe --always || echo v0.0.0)" > /opt/open_mower_install/version_info.env'

# Remove build sources and intermediate artifacts to slim image (keep only install space)
RUN rm -rf /opt/open_mower_ros

# Safe pruning: docs, man pages, locales (keep en), static libs
RUN bash -euxo pipefail <<'EOF'
rm -rf /usr/share/doc /usr/share/info || true
find /usr/share/locale -maxdepth 1 -type d ! -name 'locale' ! -name 'en_US' ! -name 'en' -exec rm -rf {} + 2>/dev/null || true
find / -type f -name '*.a' -delete 2>/dev/null || true
find / -type f -name '*.la' -delete 2>/dev/null || true
rm -rf /var/cache/apt/* /var/lib/apt/lists/*
EOF

RUN mkdir -p /config
COPY ./docker/assets/rosconsole.config /config/rosconsole.config
COPY ./docker/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

# Create openmower runtime user so that we can run the container non-root
ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && adduser ${USERNAME} dialout \
    && chown -R ${USER_UID}:${USER_GID} /config /home/${USERNAME}
RUN usermod -aG sudo ${USERNAME}
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}

ENTRYPOINT ["/openmower_entrypoint.sh", "osv2"]
CMD ["bash", "-c", "roslaunch open_mower open_mower.launch --screen"]
