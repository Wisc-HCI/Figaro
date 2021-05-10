#!/bin/bash

source /opt/ros/melodic/setup.bash

# make the workspace
cd ~/catkin_ws
rm -rf build/ devel/
catkin_make