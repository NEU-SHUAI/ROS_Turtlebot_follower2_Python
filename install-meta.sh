#!/bin/bash

# Installs basic metapackages for turtlebog

# ROS_DIST="hydro"
ROS_DIST="indigo"
# ROS_DIST="jade"

sudo apt-get install -y ros-$ROS_DIST-turtlebot. ros-$ROS_DIST-libuvc. ros-$ROS_DIST-openni.
rosdep update

exit 0
