#!/bin/bash

source ~/Documents/Figaro/install/setup.bash
source ~/ros2_eloquent/ros2-linux/setup.bash

# this is quite hacky, but ros1 cannot be in the library path
export LD_LIBRARY_PATH=/home/david/Documents/Figaro/install/robot_msgs/lib:/home/david/Documents/Figaro/install/figaro_msgs/lib:/home/david/ros2_eloquent/ros2-linux/opt/yaml_cpp_vendor/lib:/home/david/ros2_eloquent/ros2-linux/opt/rviz_ogre_vendor/lib:/usr/lib:/home/david/ros2_eloquent/ros2-linux/lib
echo $LD_LIBRARY_PATH

cd ~/Documents/Figaro/touch_tracking/touch_tracking
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/freenect2/lib
ros2 run touch_tracking controller_node $1
