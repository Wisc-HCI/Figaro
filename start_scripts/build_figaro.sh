#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/ros2_eloquent/ros2-linux/setup.bash
export ROS2_DISTRO=eloquent
cd ~/Documents/Figaro
rm -rf build/ install/ log/
colcon build --symlink-install --packages-skip ros1_bridge