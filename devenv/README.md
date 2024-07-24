# Devenv
A Dockerized version of ros-noetic-desktop-full.

## How to use
- run `./start_devenv.sh` to build and run the ROS container. This will start the container in the background, you can attach to it later.
- run `./attach.sh` to attach to the container. You should be able to do `catkin_make` and also use tools like `rviz`
