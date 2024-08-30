#!/bin/sh

. /robast/humble/setup.sh
ros2 launch nav_bringup robot_localization_odom_to_base_launch.py &
ros2 launch nav_bringup slam_toolbox_launch.py &
ros2 launch nav_bringup nav_without_localization_launch.py &
ros2 launch nav_bringup mask_server_launch.py
