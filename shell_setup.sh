#!/bin/sh
# Setup Environmental Path Variables
export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/map_server/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/aws_hospital_world/fuel_models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}

# Set robot_model and world_model:
WORLD_MODEL=hospital
export ROBOT_MODEL=rb_theron

# Inititial pose of the robot, which is important for spawning the robot in the map AND for start position for SLAM
if [ $WORLD_MODEL = "5OG" ];
then
	export WORLD_MODEL=5OG
	export POSE_INIT_X=8.59
	export POSE_INIT_Y=-13.45
	export POSE_INIT_Z=0.35
else
	export WORLD_MODEL=hospital
	export POSE_INIT_X=0.0
	export POSE_INIT_Y=10.0
	export POSE_INIT_Z=0.35
fi

# Source
source install/setup.bash
source /usr/share/gazebo/setup.sh