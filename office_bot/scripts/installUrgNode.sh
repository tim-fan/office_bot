#!/bin/bash

set -e

#create a catkin_ws in the current dir and install the hokuyo driver in it
source /opt/ros/kinetic/setup.bash

mkdir -p catkin_ws/src
pushd catkin_ws
wstool init src
rosinstall_generator urg_node --deps --exclude RPP | wstool merge -t src -
wstool update -t src
catkin_make
popd
