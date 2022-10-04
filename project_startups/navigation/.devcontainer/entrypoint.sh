#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y
ros2 param set /robot/front_laser_node angle_start -1.95
ros2 param set /robot/front_laser_node angle_end 1.95