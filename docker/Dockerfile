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
rosdep install --from-paths src --ignore-src --simulate | sed '1d' | sort | tr -d '\n' | sed 's/  apt-get install//g' > apt-install_list
apt-get update
xargs -r -a apt-install_list apt-get install --no-install-recommends -y
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
rosdep install --from-paths src --ignore-src --simulate | sed '1d' | sort | tr -d '\n' | sed 's/  apt-get install//g' > /apt-install_list
rm -rf /var/lib/apt/lists/*
EOF

####################################################################################################
# We can't derive this from "dependencies" because "dependencies" will be rebuilt every time, but apt install should only be done if needed
# Build open_mower_ros with -DCMAKE_INSTALL_PREFIX=/opt/open_mower_install (which is a copy of the binaries)
FROM base AS openmower-build

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r built earlier, this only changes if slic3r was changed (probably never)
COPY --link --from=slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner

# Fetch the list of packages, this only changes if new dependencies have been added (only sometimes)
COPY --link --from=dependencies /apt-install_list /apt-install_list
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) && \
    rm -rf /var/lib/apt/lists/*

COPY --link ./ /opt/open_mower_ros

# Delete prebuilt libs (so that they won't shadow the preinstalled ones)
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

WORKDIR /opt/open_mower_ros

RUN bash -exo pipefail <<'EOF'
source /opt/ros/$ROS_DISTRO/setup.bash
cd /opt/open_mower_ros/src
catkin_init_workspace
cd ..
source /opt/prebuilt/slic3r_coverage_planner/setup.bash
catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner -DCMAKE_INSTALL_PREFIX=/opt/open_mower_install install
EOF

RUN bash -c 'echo "export OM_SOFTWARE_VERSION=$(git describe --tags || git describe --always || echo v0.0.0)" > version_info.env'
RUN install -D -m 0644 version_info.env /opt/open_mower_install/version_info.env

# Collect runtime shared library dependencies from the install space here so
# the distilled runtime stage can import just the needed shared libs without
# having to apt-install full dependency packages.
RUN bash -exo pipefail <<'EOF'
set -e
echo "[Distill] Collecting transitive runtime libraries (openmower-build)"

ldd_abs() {
    # Print absolute library paths from ldd output (handles both formats)
    ldd "$1" 2>/dev/null | awk '{
        if ($2 == "=>" && $3 ~ /^\//) { print $3; next }
        if ($1 ~ /^\//) { print $1; next }
    }' | grep -E '^/'
}

WORK=/tmp/rtlib
rm -rf "$WORK" && mkdir -p "$WORK"
touch "$WORK/queue" "$WORK/seen" "$WORK/result"

# Seed queue with all ELF executables and shared objects from install space
find /opt/open_mower_install -type f \( -perm -111 -o -name '*.so' -o -name '*.so.*' \) -print0 | \
    xargs -0 -r file | awk -F: '/ELF/ {print $1}' > "$WORK/queue"

while read -r item; do
    [ -z "$item" ] && continue
    # Skip if we've processed this item already
    grep -Fxq "$item" "$WORK/seen" 2>/dev/null && continue
    echo "$item" >> "$WORK/seen"

    # Collect its direct deps
    deps=$(ldd_abs "$item")
    if [ -n "$deps" ]; then
        # Filter existing files only
        while read -r dep; do
            [ -z "$dep" ] && continue
            [ -e "$dep" ] || continue
            echo "$dep" >> "$WORK/result"
            # Enqueue dep for transitive closure if not seen
            grep -Fxq "$dep" "$WORK/seen" 2>/dev/null || echo "$dep" >> "$WORK/queue"
            # If dep is symlink, add realpath as well
            if [ -L "$dep" ]; then
                rp=$(readlink -f "$dep")
                [ -n "$rp" ] && [ -e "$rp" ] && echo "$rp" >> "$WORK/result"
                grep -Fxq "$rp" "$WORK/seen" 2>/dev/null || echo "$rp" >> "$WORK/queue"
            fi
        done <<< "$deps"
    fi
done < "$WORK/queue"

sort -u "$WORK/result" > /tmp/runtime_libs_list

if [ -s /tmp/runtime_libs_list ]; then
    tar -C / -cf /tmp/runtime_libs.tar $(cat /tmp/runtime_libs_list)
    echo "[Distill] Collected $(wc -l < /tmp/runtime_libs_list) libraries"
else
    echo "[Distill] No runtime libs list produced (possibly empty install space?)"
fi
EOF

####################################################################################################

# Assemble final OSv1 image and add nginx/mosquitto
FROM base AS assemble-osv1

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r built earlier, this only changes if slic3r was changed (probably never)
COPY --link --from=slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner

# Fetch the list of packages, this only changes if new dependencies have been added (only sometimes)
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

# Delete prebuilt libs (so that they won't shadow the preinstalled ones)
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

WORKDIR /opt/open_mower_ros

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /opt/open_mower_ros/src && catkin_init_workspace && cd .. && source /opt/prebuilt/slic3r_coverage_planner/setup.bash && catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner"
RUN echo "export OM_SOFTWARE_VERSION=$(git describe --tags || git describe --always || echo "v0.0.0")" > version_info.env

COPY ./docker/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

ENTRYPOINT ["/openmower_entrypoint.sh", "osv1"]
CMD ["bash", "-c", "service nginx start; service mosquitto start; roslaunch open_mower open_mower.launch --screen"]


####################################################################################################

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

# Delete prebuilt libs (so that they won't shadow the preinstalled ones)
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

WORKDIR /opt/open_mower_ros

# Single install build (skip separate devel build). Creates devel + install, we later discard sources.
RUN bash -exo pipefail <<'EOF'
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/prebuilt/slic3r_coverage_planner/setup.bash
cd /opt/open_mower_ros/src && catkin_init_workspace
cd /opt/open_mower_ros
catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner -DCMAKE_INSTALL_PREFIX=/opt/open_mower_install install
EOF

# Build-time sanity check: ensure key packages resolvable from install space
#RUN bash -exo pipefail <<'EOF'
#source /opt/ros/$ROS_DISTRO/setup.bash
#source /opt/open_mower_install/setup.bash || true
#rospack find open_mower > /dev/null 2>&1 || { echo 'ERROR: open_mower package not found in install space'; exit 1; }
#EOF

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

# Ensure temporary directories have correct sticky-bit permissions
RUN mkdir -p /tmp /var/tmp && chmod 1777 /tmp /var/tmp


# Simplified (stabilized) slimming: remove only non-critical weight (docs/locales/static libs).
# Aggressive apt purging temporarily disabled to avoid removing ROS runtime.
ARG RUNTIME_FULL=0
ARG RUNTIME_EXTENDED_PRUNE=0  # additional file-level pruning (no package purges)
ARG RUNTIME_HEAVY_PRUNE=0     # remove large optional libs (LLVM/Clang/WebKit/ICU/GDAL/PCL/DRI/x265) if not needed
RUN set -eux; \
    if [ "$RUNTIME_FULL" != "1" ]; then \
    echo "[Slim] Base cleanup (docs/locales/static libs)"; \
    rm -rf /usr/share/doc /usr/share/info /usr/share/man || true; \
    find /usr/share/locale -maxdepth 1 -type d ! -name 'locale' ! -name 'en_US' ! -name 'en' -exec rm -rf {} + 2>/dev/null || true; \
    find / -type f -name '*.a' -delete 2>/dev/null || true; \
    find / -type f -name '*.la' -delete 2>/dev/null || true; \
    fi; \
    if [ "$RUNTIME_FULL" != "1" ] && [ "$RUNTIME_EXTENDED_PRUNE" = "1" ]; then \
    echo "[Slim][Extended] Removing headers"; rm -rf /usr/include/* 2>/dev/null || true; \
    echo "[Slim][Extended] Removing pkgconfig + cmake metadata"; \
    rm -rf /usr/lib/pkgconfig /usr/share/pkgconfig /usr/lib/cmake /usr/share/cmake* /opt/ros/$ROS_DISTRO/lib/pkgconfig /opt/ros/$ROS_DISTRO/lib/cmake 2>/dev/null || true; \
    echo "[Slim][Extended] Removing python tests & caches"; \
    find /usr/lib/python3/dist-packages -type d -name tests -prune -exec rm -rf {} + 2>/dev/null || true; \
    find /usr/lib/python3/dist-packages -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true; \
    find /usr/lib/python3/dist-packages -type f -name '*.pyc' -delete 2>/dev/null || true; \
    echo "[Slim][Extended] Stripping selected binaries"; \
    if command -v strip >/dev/null 2>&1; then \
    find /opt/open_mower_install -type f -size +80k -exec sh -c 'file "$1" | grep -q ELF && strip --strip-unneeded "$1" 2>/dev/null || true' _ {} \; 2>/dev/null || true; \
    fi; \
    fi; \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/* 2>/dev/null || true

# Heavy prune: only when slim (RUNTIME_FULL!=1) and explicitly enabled
RUN set -eux; \
    if [ "$RUNTIME_FULL" != "1" ] && [ "$RUNTIME_HEAVY_PRUNE" = "1" ]; then \
    echo "[HeavyPrune] Removing Mesa DRI drivers"; rm -rf /usr/lib/x86_64-linux-gnu/dri 2>/dev/null || true; \
    echo "[HeavyPrune] Removing LLVM/Clang toolchain shared libs"; \
    rm -f /usr/lib/x86_64-linux-gnu/libLLVM-*.so.* 2>/dev/null || true; \
    rm -rf /usr/lib/llvm-* 2>/dev/null || true; \
    rm -f /usr/lib/x86_64-linux-gnu/libclang-cpp.so.* /usr/lib/x86_64-linux-gnu/libclang-*.so.* 2>/dev/null || true; \
    echo "[HeavyPrune] Removing Qt WebKit + ICU data if present"; \
    rm -f /usr/lib/x86_64-linux-gnu/libQt5WebKit.so.* 2>/dev/null || true; \
    rm -f /usr/lib/x86_64-linux-gnu/libicudata.so.* 2>/dev/null || true; \
    echo "[HeavyPrune] Removing GDAL (geospatial)"; rm -f /usr/lib/libgdal.so.* 2>/dev/null || true; \
    echo "[HeavyPrune] Removing PCL heavy feature libs"; find /usr/lib -maxdepth 1 -type f -name 'libpcl_*.so*' -delete 2>/dev/null || true; \
    echo "[HeavyPrune] Removing x265 codec"; rm -f /usr/lib/x86_64-linux-gnu/libx265.so.* 2>/dev/null || true; \
    echo "[HeavyPrune] Size after removal:"; du -sh /usr/lib || true; \
    else \
    echo "[HeavyPrune] Skipped (RUNTIME_HEAVY_PRUNE!=1 or RUNTIME_FULL=1)"; \
    fi

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

####################################################################################################
# Minimal distilled runtime stage (copy only install space + required shared libs)
FROM docker.io/ros:noetic-ros-core-focal AS runtime-osv2

ENV DEBIAN_FRONTEND=noninteractive

# Copy install artifacts, prebuilt planner and collected runtime libs from build stage
COPY --link --from=openmower-build /opt/open_mower_install /opt/open_mower_install
COPY --link --from=openmower-build /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner
COPY --link --from=openmower-build /tmp/runtime_libs.tar /tmp/runtime_libs.tar

# Extract runtime libs collected during build
RUN tar -xf /tmp/runtime_libs.tar -C / && rm -f /tmp/runtime_libs.tar

# Ensure dynamic linker cache is updated to see copied libraries (incl. ROS paths)
RUN bash -euxo pipefail <<'EOF'
echo "/opt/ros/$ROS_DISTRO/lib" > /etc/ld.so.conf.d/ros.conf
# Default /lib and /usr/lib trees are already configured by base image; just refresh cache
ldconfig || true
EOF

# Ensure temporary directories have correct sticky-bit permissions
RUN mkdir -p /tmp /var/tmp && chmod 1777 /tmp /var/tmp

# Aggressive but safe runtime prune (no compilers here, remove dev & locales not needed)
#RUN bash -euxo pipefail <<'EOF'
#rm -rf /usr/share/doc /usr/share/info /usr/share/man || true
#find /usr/share/locale -maxdepth 1 -type d ! -name 'locale' ! -name 'en_US' ! -name 'en' -exec rm -rf {} + 2>/dev/null || true
#find / -type f -name '*.a' -delete 2>/dev/null || true
#find / -type f -name '*.la' -delete 2>/dev/null || true
#rm -rf /usr/include || true
#rm -rf /usr/lib/pkgconfig /usr/share/pkgconfig /usr/lib/cmake /usr/share/cmake* || true
# Mesa DRI (should not be in ros-core, but defensive)
#rm -rf /usr/lib/x86_64-linux-gnu/dri 2>/dev/null || true
#rm -rf /var/lib/apt/lists/* /var/cache/apt/* || true
#EOF

# Optional toolchain / sanitizer / ICU data prune (only if explicitly enabled)
#RUN bash -euxo pipefail <<'EOF'
#if [ "${RUNTIME_PRUNE_TOOLCHAIN}" = "1" ]; then
#    echo "[ToolchainPrune] Removing LLVM/Clang toolchain, sanitizers, SBCL core, optional ICU data";
# LLVM / Clang trees & libs (match also unversioned symlink forms)
#    rm -rf /usr/lib/llvm-* 2>/dev/null || true
#    rm -f /usr/lib/x86_64-linux-gnu/libclang-cpp.so* /usr/lib/x86_64-linux-gnu/libclang-*.so* 2>/dev/null || true
#    rm -f /usr/lib/x86_64-linux-gnu/libLLVM-*.so* 2>/dev/null || true
# Sanitizers
#    rm -f /usr/lib/x86_64-linux-gnu/libasan.so.* /usr/lib/x86_64-linux-gnu/libtsan.so.* /usr/lib/x86_64-linux-gnu/liblsan.so.* 2>/dev/null || true
# Lisp core (rarely needed at runtime)
#    rm -rf /usr/lib/sbcl 2>/dev/null || true
# ICU data only if not referenced OR forced
#    if [ "${FORCE_REMOVE_ICU}" = "1" ]; then
#        echo "[ToolchainPrune] FORCE_REMOVE_ICU=1 -> dropping ICU data"
#        rm -f /usr/lib/x86_64-linux-gnu/libicudata.so* 2>/dev/null || true
#    else
#       if ! grep -R "icudata" /opt/open_mower_install 2>/dev/null | head -n1 | grep -q icudata; then
#           rm -f /usr/lib/x86_64-linux-gnu/libicudata.so* 2>/dev/null || true
#       else
#           echo "[ToolchainPrune] ICU data referenced in install tree -> keeping";
#       fi
#   fi
#   echo "[ToolchainPrune] After removal largest libs:";
#   find /usr/lib -type f -name '*.so*' -size +8M -printf '%9s %p\n' 2>/dev/null | sort -hr | head -n 20 || true
#else
#    echo "[ToolchainPrune] Skipped (RUNTIME_PRUNE_TOOLCHAIN!=1)";
#fi
#EOF

RUN mkdir -p /config
COPY ./docker/assets/rosconsole.config /config/rosconsole.config
COPY ./docker/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && adduser ${USERNAME} dialout \
    && chown -R ${USER_UID}:${USER_GID} /config /home/${USERNAME}
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}

ENTRYPOINT ["/openmower_entrypoint.sh", "osv2"]
CMD ["bash", "-c", "roslaunch open_mower open_mower.launch --screen"]
