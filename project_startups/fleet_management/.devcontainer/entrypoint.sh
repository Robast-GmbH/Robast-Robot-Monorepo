#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src/open_rmf --ignore-src -r -y
rosdep install --from-paths /workspace/src/communication_interfaces --ignore-src -r -y