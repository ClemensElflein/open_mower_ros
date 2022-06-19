FROM ros:noetic-ros-base

RUN rosdep update
RUN apt update && \
    apt install -y git

RUN git clone --recurse-submodules https://github.com/ClemensElflein/OpenMower /opt/openmower

WORKDIR /opt/openmower/ROS
RUN rosdep install --from-paths src --ignore-src -y
RUN bash -c "source "/opt/ros/$ROS_DISTRO/setup.bash" && catkin_make"

COPY .github/assets/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

ENTRYPOINT ["/openmower_entrypoint.sh"]
CMD ["roslaunch", "open_mower", "open_mower.launch"]
