#!/bin/bash

# run this script to compile a local open_mower_ros repository using the tools contained
# in the docker image. This is a good idea, because this way we don't need to install any tooling
# and are independent of any host OS.

docker run \
	-v $HOME/mower_config.sh:/config/mower_config.sh \
	-v $HOME/open_mower_ros:/opt/open_mower_ros\
	--device /dev/ttyAMA0:/dev/ttyAMA0\
        --device /dev/ttyAMA1:/dev/ttyAMA1\
        --device /dev/ttyAMA2:/dev/ttyAMA2\
        --device /dev/ttyAMA3:/dev/ttyAMA3\
        --device /dev/ttyAMA4:/dev/ttyAMA4\
	--network="host"\
	--entrypoint=\
	-it\
	ghcr.io/clemenselflein/open_mower_ros:releases-testing\
	bash -c "source /opt/ros/noetic/setup.bash && cd /opt/open_mower_ros && source /opt/prebuilt/slic3r_coverage_planner/setup.bash && catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner"
