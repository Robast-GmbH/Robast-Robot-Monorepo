#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
pip install yolov5 # TODO remove before merging to main
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y
