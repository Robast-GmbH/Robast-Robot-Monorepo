#!/bin/bash
mkdir -p ~/rviz2_ws/src
cd ~/rviz2_ws/src
git clone https://github.com/ros2/rviz.git
cp -r ./resource_retriever ~/rviz2_ws/src/rviz/rviz_rendering/include/
colcon build --merge-install
cd /workspaces/foxy_ws

#sudo /startup.sh
