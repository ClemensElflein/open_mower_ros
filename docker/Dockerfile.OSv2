# Lean image without nginx and mosquitto for OSv2 (OS-level compose provides services)
FROM docker.io/ros:noetic-ros-base-focal AS base
ENV DEBIAN_FRONTEND=noninteractive

# Install required base packages only (no nginx/mosquitto here)
RUN apt-get update && apt-get install --yes \
    git \
    python3.8-venv \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update --rosdistro $ROS_DISTRO

####################################################################################################

# Build slic3r_coverage_planner once and re-use the prebuilt install
FROM base AS slic3r

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r planner from the repo (this will cache if unchanged)
COPY --link src/lib/slic3r_coverage_planner /opt/slic3r_coverage_planner_workspace/src

WORKDIR /opt/slic3r_coverage_planner_workspace
RUN rosdep install --from-paths src --ignore-src --simulate | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > apt-install_list && \
    apt-get update && apt-get install --no-install-recommends --yes $(cat apt-install_list) libasio-dev && \
    rm -rf /var/lib/apt/lists/* apt-install_list
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /opt/slic3r_coverage_planner_workspace/devel/setup.bash && catkin_make -DCMAKE_INSTALL_PREFIX=/opt/prebuilt/slic3r_coverage_planner install"

####################################################################################################

# Prepare dependency list from workspace
FROM base AS dependencies

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /opt/open_mower_ros

RUN --mount=type=bind,source=src,target=src \
    apt-get update && \
    rosdep install --from-paths src --ignore-src --simulate | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list \
    && rm -rf /var/lib/apt/lists/*

####################################################################################################

# Assemble final image (without nginx/mosquitto)
FROM base AS assemble

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r built earlier
COPY --link --from=slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner

# Fetch the list of packages
COPY --link --from=dependencies /apt-install_list /apt-install_list
RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) libasio-dev && \
    rm -rf /var/lib/apt/lists/*

COPY --link ./ /opt/open_mower_ros

# delete prebuilt libs (so that they won't shadow the preinstalled ones)
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

WORKDIR /opt/open_mower_ros

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /opt/open_mower_ros/src && catkin_init_workspace && cd .. && source /opt/prebuilt/slic3r_coverage_planner/setup.bash && catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner"

RUN echo "export OM_SOFTWARE_VERSION=$(git describe --tags || git describe --always || echo \"v0.0.0\")" > version_info.env

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
    && chown -R ${USER_UID}:${USER_GID} /config /home/${USERNAME}

# Append our bashrc extension to user's .bashrc
COPY ./docker/assets/openmower-bashrc.sh /tmp/openmower-bashrc.sh
RUN set -eux; \
    BRC="/home/${USERNAME}/.bashrc"; \
    touch "$BRC"; \
    chown ${USER_UID}:${USER_GID} "$BRC"; \
    printf '\n# OpenMower bashrc extension\n' >> "$BRC"; \
    cat /tmp/openmower-bashrc.sh >> "$BRC"; \
    rm -f /tmp/openmower-bashrc.sh

USER ${USERNAME}

ENTRYPOINT ["/openmower_entrypoint.sh", "osv2"]
CMD ["bash", "-c", "roslaunch open_mower open_mower.launch --screen"]
