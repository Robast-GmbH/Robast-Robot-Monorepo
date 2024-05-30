#!/bin/sh

. /robast/humble/setup.sh
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0 &
ros2 launch drawer_bridge drawer_bridge_launch.py