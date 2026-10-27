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

# Build the soundctl Go binary. OM_SOFTWARE_VERSION is injected so that
# `soundctl version` and the "generator" field of the tts sidecars name the image
# they came from (the assemble stage declares the same build arg, CI passes it once).
FROM golang:1.23 AS gobuild
ARG OM_SOFTWARE_VERSION=dev
WORKDIR /src
COPY utils/soundctl/go.mod utils/soundctl/go.sum ./
RUN go mod download
COPY utils/soundctl/ ./
RUN CGO_ENABLED=0 go build -trimpath -ldflags "-s -w -X github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/cli.version=${OM_SOFTWARE_VERSION}" -o /out/soundctl ./cmd/soundctl

# Assemble final image (without nginx/mosquitto)
FROM base AS assemble

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the rosserial and slic3r built earlier
COPY --link --from=rosserial /opt/prebuilt/rosserial /opt/prebuilt/rosserial
COPY --link --from=slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner

# Fetch the list of packages
COPY --link --from=dependencies /apt-install_list /apt-install_list
# ffmpeg for `soundctl tts` (loudnorm + MP3 encoding) and ca-certificates for the
# HTTPS downloads of the TTS engines it installs.
RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) libasio-dev ffmpeg ca-certificates && \
    rm -rf /var/lib/apt/lists/*

COPY --link ./ /opt/open_mower_ros
COPY --link --from=gobuild /out/soundctl /usr/local/bin/soundctl

# TTS engine + model for the `tts:` texts of the sound definitions
RUN soundctl tts install --dir /opt/open_mower_ros/tts --arch "$(dpkg --print-architecture)"

# Gate the shipped definitions here rather than on the mower: ids/types/ranges
# against services/sound_service.json and 16 kHz mono for every referenced MP3.
# The announcements come from the CI pre-render in the build context (see
# .github/workflows/build-image.yaml); a language whose render is missing would
# be reported as "pending" instead of failing, so this stays warning-free either way.
RUN for lang in en de; do \
    soundctl validate \
    --config "/opt/open_mower_ros/src/open_mower/params/sounds_${lang}.yaml" \
    --check-mp3; \
    done

# Piper engine + the English voice (sounds_en.yaml renders with engine: piper).
# soundctl pins both by SHA256 (internal/tts/install.go). The 2023 standalone build
# is used on purpose: no Python in the image, ~4x faster than the pip CLI, and it
# loads the current voices.
RUN soundctl tts install --engine piper --dir /opt/open_mower_ros/tts && \
    soundctl tts verify --engine piper --dir /opt/open_mower_ros/tts

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
