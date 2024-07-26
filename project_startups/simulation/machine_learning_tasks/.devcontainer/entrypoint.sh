#!/bin/bash
pip install -r src/machine_learning_tasks/requirements.txt
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y