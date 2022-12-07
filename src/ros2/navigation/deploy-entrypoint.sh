#!/bin/bash
cd /workspace; 
source install/setup.bash; 

ros2 launch nav_bringup slam_toolbox_launch.py&
ros2 launch nav_bringup nav_without_localization_launch.py&

if [ -z ${DOCKER_MODE+x} ]; then
    tail -f /dev/null
 fi