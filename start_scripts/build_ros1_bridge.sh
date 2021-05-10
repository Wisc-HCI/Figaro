#!/bin/bash


source ~/ros2_eloquent/ros2-linux/setup.bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

cd ~/Documents/Figaro
source install/setup.bash

export CMAKE_PREFIX_PATH=/home/david/Documents/Figaro/install/robot_msgs:/home/david/ros2_eloquent/ros2-linux:/opt/ros/melodic:/home/david/catkin_ws/devel

colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

