#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
pip install yolov5 # TODO remove before merging to main
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y

rosdep install --from-paths /mppi --ignore-src -r -y # TODO remove this line when mppi works from package install
cd /mppi; colcon build --packages-select nav2_mppi_controller # TODO remove this line when mppi works from package install
