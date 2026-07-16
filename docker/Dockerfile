# Lean image without nginx and mosquitto for OSv2 (OS-level compose provides services)
FROM docker.io/ros:noetic-ros-base-focal AS base
ENV DEBIAN_FRONTEND=noninteractive

# Install required base packages only (no nginx/mosquitto here)
RUN apt-get update && apt-get install --yes \
    git \
    python3.8-venv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update --rosdistro $ROS_DISTRO

####################################################################################################

# Build rosserial once and re-use the prebuilt install
FROM base AS rosserial

ENV DEBIAN_FRONTEND=noninteractive

# Clone from a fixed commit hash
RUN mkdir -p /opt/rosserial/src && cd /opt/rosserial/src && git clone --no-checkout https://github.com/tongtybj/rosserial && cd rosserial && git fetch --depth=1 origin 47a73176d281eb7cc582e6dc1eb83ada40cb5f8d && git checkout FETCH_HEAD

WORKDIR /opt/rosserial/
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make_isolated --install --install-space /opt/prebuilt/rosserial --only-pkg-with-deps rosserial_server"

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

# Fetch the rosserial and slic3r built earlier
COPY --link --from=rosserial /opt/prebuilt/rosserial /opt/prebuilt/rosserial
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

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /opt/open_mower_ros/src && catkin_init_workspace && cd .. && source /opt/prebuilt/slic3r_coverage_planner/setup.bash && source /opt/prebuilt/rosserial/setup.bash --extend && catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner"

# Version is passed as a build-arg from CI (the build context excludes .git/, so an
# in-container `git describe` can't resolve it). Fall back to git describe for local
# builds that include .git/, then to v0.0.0.
ARG OM_SOFTWARE_VERSION=""
RUN echo "export OM_SOFTWARE_VERSION=${OM_SOFTWARE_VERSION:-$(git describe --tags 2>/dev/null || git describe --always 2>/dev/null || echo v0.0.0)}" > version_info.env

RUN mkdir -p /config
COPY ./docker/assets/rosconsole.config /config/rosconsole.config
COPY ./docker/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

# Create openmower runtime user so that we can run the container non-root
ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    groupadd -g 996 input \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && adduser ${USERNAME} dialout \
    && adduser ${USERNAME} input \
    && chown -R ${USER_UID}:${USER_GID} /config /home/${USERNAME}

# Append our bashrc extension to user's .bashrc
COPY ./docker/assets/openmower-bashrc.sh /tmp/openmower-bashrc.sh
RUN set -eux; \
    BRC="/home/${USERNAME}/.bashrc"; \
    # Uncomment force_color_prompt
    grep -q '^#force_color_prompt=yes' "$BRC" && sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' "$BRC"; \
    chown ${USER_UID}:${USER_GID} "$BRC"; \
    printf '\n# OpenMower bashrc extension\n' >> "$BRC"; \
    cat /tmp/openmower-bashrc.sh >> "$BRC"; \
    rm -f /tmp/openmower-bashrc.sh
RUN usermod -aG sudo ${USERNAME}
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}

ENTRYPOINT ["/openmower_entrypoint.sh"]
CMD ["bash", "-c", "roslaunch open_mower open_mower.launch --screen"]
