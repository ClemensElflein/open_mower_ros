#!/bin/bash
echo "Starting OpenMower Container"
/opt/ros/noetic/bin/roscore &
/opt/ros/noetic/bin/rosrun rosserial_server serial_node _port:=/dev/mowgli _baud:=115200 &
/opt/ros/noetic/bin/roslaunch open_mower open_mower.launch --screen --wait
