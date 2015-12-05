#!/bin/bash

# Script to setup a catkin workspace with the repo files

SCRIPT_DIR="$(cd "$(dirname"${BASH_SOURCE[0]}")" && pwd)"

# source the environment
. /opt/ros/indigo/setup.bash

# go to repo's directory
pushd "$SCRIPT_DIR/turtlebot_gt"

# setup the source files
catkin_init_workspace

