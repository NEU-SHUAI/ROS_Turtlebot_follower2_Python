#!/bin/bash

# Script to setup a catkin workspace with the repo files

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# source the environment
. /opt/ros/indigo/setup.bash

# go to repo's directory
pushd "$SCRIPT_DIR/src" &> /dev/null

# setup the source files if necessary
if ! catkin_init_workspace &> /dev/null; then
    echo "--  no catkin initialization needed"
fi

pushd .. &> /dev/null
echo "--  attempting to build package"
# any failure after now ends the script
set -e
catkin_make

# blank line after build output
echo ""

# try to make the package and ask to
# add the sourcing env to the user's 
# bashrc file
if ! grep -q " $(readlink -m $(pwd)/devel/setup.bash)" ~/.bashrc-test; then
    read -r -p "Would you like add the env to ~/.bashrc? [y/N] " response
    case $response in
        [yY][eE][sS]|[yY]) 
            echo ". $(readlink -m $(pwd)/devel/setup.bash)" >> ~/.bashrc-test
            echo "--  added new ros env to ~/.bashrc"
        ;;
    *)
        :
        ;;
    esac
fi

exit 0
