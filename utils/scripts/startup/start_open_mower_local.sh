#!/bin/bash

# runs the code in the local repository. I use this for testing small changes quickly on the mower.
# First compile and then run this script to run.

docker run \
	-v $HOME/mower_config.sh:/config/mower_config.sh \
	-v $HOME/open_mower_ros:/opt/open_mower_ros\
	-v $HOME/.ros:/root/.ros\
	--device /dev/ttyAMA0:/dev/ttyAMA0\
        --device /dev/ttyAMA1:/dev/ttyAMA1\
        --device /dev/ttyAMA2:/dev/ttyAMA2\
        --device /dev/ttyAMA3:/dev/ttyAMA3\
        --device /dev/ttyAMA4:/dev/ttyAMA4\
	--network="host"\
	-t\
	ghcr.io/clemenselflein/open_mower_ros:releases-testing
