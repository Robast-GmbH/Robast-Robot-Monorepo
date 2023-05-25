#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y
rosdep install --from-paths /workspace_nav2/src --ignore-src -r -y
cd /workspace_nav2; colcon build --packages-select nav2_mppi_controller

