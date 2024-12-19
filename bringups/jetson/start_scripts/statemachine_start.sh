#!/bin/sh

. /robast/humble/setup.sh
ros2 launch drawer_sm electrical_drawer_launch.py &
ros2 launch drawer_sm robot_base_led_bt_launch.py &
ros2 launch drawer_sm partial_drawer_launch.py &
ros2 topic pub /trigger_base_tree communication_interfaces/msg/DrawerAddress '{module_id: 1, drawer_id: 0}' --once &
ros2 launch drawer_sm heartbeat_launch.py &
ros2 topic pub /trigger_heartbeat_tree std_msgs/msg/Empty --once &
ros2 launch drawer_sm drawer_statemachine_launch.py