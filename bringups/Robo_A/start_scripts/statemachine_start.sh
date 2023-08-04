#!/bin/sh

. /robast/humble/setup.sh
ros2 launch drawer_sm electrical_drawer_launch.py &
ros2 launch drawer_sm drawer_statemachine_launch.py