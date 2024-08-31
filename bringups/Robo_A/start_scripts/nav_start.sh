#!/bin/sh

. /robast/humble/setup.sh
ros2 param set /robot/front_laser_node angle_start -1.95
ros2 param set /robot/front_laser_node angle_end 1.95
ros2 launch nav_bringup robot_localization_odom_to_base_launch.py &
ros2 launch nav_bringup slam_toolbox_launch.py &
ros2 launch nav_bringup nav_without_localization_launch.py &
ros2 launch nav_bringup mask_server_launch.py &
ros2 launch nav_action_node nav_action_node.launch.py &
ros2 launch robot_pose_publisher robot_pose_publisher.launch.py &
ros2 launch clicked_point_to_nav_pose clicked_point_to_nav_pose_launch.py
