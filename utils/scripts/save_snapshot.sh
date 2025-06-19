#!/bin/sh

# This script saves a record of all recent low-level ros messages to a ros bag.
# Usage: ./save_snapshot.sh output_file.bag


rosrun rosbag_snapshot snapshot -t -O "$1"
