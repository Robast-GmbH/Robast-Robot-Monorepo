#!/bin/sh
# Setup der Variablen
export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/map_server/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}
export WORLD_MODEL=5OG
export ROBOT_MODEL=rb_theron

# Inititial pose of the robot, which is important for spawning the robot in the map AND for start position for SLAM
export POSE_INIT_X=8.59
export POSE_INIT_Y=-13.45
export POSE_INIT_Z=0.35

# Source
source install/setup.bash
source /usr/share/gazebo/setup.sh