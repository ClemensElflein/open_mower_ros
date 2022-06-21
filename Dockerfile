FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN rosdep update
RUN apt update && \
    apt install -y git

COPY ./ /opt/openmower

WORKDIR /opt/openmower

RUN git submodule update --init --recursive

RUN rosdep install --from-paths src --ignore-src -y
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

COPY .github/assets/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

ENTRYPOINT ["/openmower_entrypoint.sh"]
CMD ["roslaunch", "open_mower", "open_mower.launch"]
