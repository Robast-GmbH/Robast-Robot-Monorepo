#!/bin/bash
export PATH=$PATH:/home/robast/.local/bin
source ~/.bashrc
pip install -r src/machine_learning_tasks/requirements.txt
pip install -r src/map_inapinting/requirements.txt
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y