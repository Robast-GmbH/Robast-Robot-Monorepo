#!/bin/sh
# Setup der Variablen
export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/map_server/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}
export WORLD_MODEL=5OG
export TURTLEBOT3_MODEL=waffle
source install/setup.bash
source /usr/share/gazebo/setup.sh