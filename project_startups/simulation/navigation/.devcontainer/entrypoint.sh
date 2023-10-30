#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y

rosdep install --from-paths /mppi --ignore-src -r -y # TODO remove this line when mppi works from package install
cd /mppi; colcon build --packages-select nav2_lifecycle_manager nav2_voxel_grid nav2_common nav2_costmap_2d nav2_core nav2_msgs nav2_util nav2_mppi_controller # TODO remove this line when mppi works from package install
