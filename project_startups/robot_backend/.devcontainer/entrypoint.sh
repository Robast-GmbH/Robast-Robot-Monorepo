#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src/ --ignore-src -r -y
pip3 install -r /workspace/src/robot_backend/requirements.txt