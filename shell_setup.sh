#!/bin/sh
# Setup Environmental Path Variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/simulation/tiplu_world/models/furniture:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}

# For Ubuntu 20.04 users, there’s a known issue with OpenVDB and its binaries as of July 2020 with libjmalloc.
# If you see an error such as Could not load library LoadLibrary error:
# /usr/lib/x86_64-linux-gnu/libjemalloc.so.2: cannot allocate memory in static TLS block,
# it can be resolved with export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so.2 until new binaries are released of OpenVDB.
# Reference: https://navigation.ros.org/tutorials/docs/navigation2_with_stvl.html
#export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so.2

# Set robot_model and world_model:
# WORLD_MODEL=5OG #aws_hospital_world or 5OG or 5OG_with_furniture
# export ROBOT_MODEL=rb_theron #rb_theron or turtlebot3_waffle

# # Inititial pose of the robot, which is important for spawning the robot in the map AND for start position for SLAM
# if [ $WORLD_MODEL = "5OG" ] || [ $WORLD_MODEL = "5OG_with_furniture" ];
# then
# 	export WORLD_MODEL=$WORLD_MODEL
# 	export POSE_INIT_X=8.59
# 	export POSE_INIT_Y=-13.45
# 	export POSE_INIT_Z=0.35
# else
# 	export WORLD_MODEL=$WORLD_MODEL
# 	export POSE_INIT_X=0.0
# 	export POSE_INIT_Y=10.0
# 	export POSE_INIT_Z=0.35
# fi

# Source
source install/setup.bash
source /usr/share/gazebo/setup.sh