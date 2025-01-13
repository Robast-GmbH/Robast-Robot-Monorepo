#!/bin/sh

. /robast/humble/setup.sh
ros2 launch nav_bringup slam_toolbox_launch.py &
ros2 launch nav_bringup nav_without_localization_launch.py &
ros2 launch nav_bringup mask_server_launch.py &
ros2 launch nav_action_node nav_action_node.launch.py &
ros2 launch robot_pose_publisher robot_pose_publisher.launch.py &
ros2 launch clicked_point_to_nav_pose clicked_point_to_nav_pose_launch.py &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
ros2 run save_pose save_map --ros-args -p file_path:="/logs/last_pose.yaml"
