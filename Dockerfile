# Get an image with git and apt-get update
FROM docker.io/ros:noetic-ros-base-focal as git
ENV DEBIAN_FRONTEND=noninteractive

RUN rosdep update
RUN apt-get update && \
    apt-get install --yes git


# First stage: Pull the git and all submodules, other stages depend on it
FROM git as fetch

ENV DEBIAN_FRONTEND=noninteractive

COPY --link ./ /opt/open_mower_ros
#RUN git clone https://github.com/ClemensElflein/open_mower_ros /opt/open_mower_ros

WORKDIR /opt/open_mower_ros

RUN git submodule update --init --recursive



# Fetch the repo and assemble the list of dependencies. We will pull these in the next step and actually run install on them
# If the package list is the same as last time, the apt install step is cached as well which saves a lot of time.
# Since the list gets sorted, it will be the same each time and the cache will know that by file checksum in the COPY command.
# We can't use this stage as base for the next, because this stage is run every time and would therefore invalidate the cache of the next stage.
FROM fetch as dependencies

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /opt/open_mower_ros

# This creates the sorted list of apt-get install commands.
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --simulate | \
    sed --expression '1d' --expression 's/apt-get install/apt-get install --no-install-recommends --yes/g' | sort > /apt-install.sh


# We can't derive this from "dependencies" because "dependencies" will be rebuilt every time, but apt install should only be done if needed
FROM git as assemble

ENV DEBIAN_FRONTEND=noninteractive

#Fetch the list of packages, this only changes if new dependencies have been added (only sometimes)
COPY --link --from=dependencies /apt-install.sh /apt-install.sh
RUN apt-get update && \
    bash /apt-install.sh && \
    rm -rf /var/lib/apt/lists/*

# This will already have the submodules initialized, no need to clone again
COPY --link --from=fetch /opt/open_mower_ros /opt/open_mower_ros


#RUN git clone https://github.com/ClemensElflein/open_mower_ros /opt/open_mower_ros

WORKDIR /opt/open_mower_ros

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /opt/open_mower_ros && catkin_make"

COPY .github/assets/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

ENTRYPOINT ["/openmower_entrypoint.sh"]
CMD ["roslaunch", "open_mower", "open_mower.launch", "--screen"]
