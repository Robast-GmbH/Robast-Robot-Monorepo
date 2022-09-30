#!/bin/bash
#mkdir -p ~/rviz2_ws/src
#cd ~/rviz2_ws/src
#git clone https://github.com/ros2/rviz.git
#cp -r ./resource_retriever ~/rviz2_ws/src/rviz/rviz_rendering/include/
#colcon build --merge-install
cd ~/../../workspaces/Robast_RosTheron
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths src --ignore-src -r -y
#sudo dos2unix shell_setup.sh
#sudo /startup.sh

export GAZEBO_MODEL_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=/workspace/src/simulation/tiplu_world/models/furniture:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/workspace/install/gazebo_plugins:${GAZEBO_PLUGIN_PATH}