#!/bin/bash

# Installs basic metapackages for turtlebog

ROS_DIST="indigo"

sudo apt-get install -y ros-$ROS_DIST-libuvc. ros-$ROS_DIST-turtlebot-interactions

rosdep update
