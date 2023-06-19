#!/bin/bash
#mkdir -p ~/rviz2_ws/src
#cd ~/rviz2_ws/src
#git clone https://github.com/ros2/rviz.git
#cp -r ./resource_retriever ~/rviz2_ws/src/rviz/rviz_rendering/include/
#colcon build --merge-instal
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y
rosdep install --from-paths /workspace_nav2/src --ignore-src -r -y # TODO remove this line
cd /workspace_nav2; colcon build --packages-select nav2_mppi_controller # TODO remove this line
#sudo dos2unix shell_setup.sh
#sudo /startup.sh
