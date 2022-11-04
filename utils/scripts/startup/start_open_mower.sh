#!/bin/bash
# Fetches the latest testing image and runs it.

docker pull ghcr.io/clemenselflein/open_mower_ros:releases-testing

docker run \
	-v $HOME/mower_config.sh:/config/mower_config.sh \
	-v $HOME/.ros:/root/.ros\
	--device /dev/ttyAMA0:/dev/ttyAMA0\
        --device /dev/ttyAMA1:/dev/ttyAMA1\
        --device /dev/ttyAMA2:/dev/ttyAMA2\
        --device /dev/ttyAMA3:/dev/ttyAMA3\
        --device /dev/ttyAMA4:/dev/ttyAMA4\
	--network="host"\
	-t\
	ghcr.io/clemenselflein/open_mower_ros:releases-testing
