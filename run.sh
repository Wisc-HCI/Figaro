#!/bin/bash

#rm -r build/ install/
colcon build
cd ctrl/ctrl
ros2 run ctrl controller_node
