#!/bin/bash
echo 'export PATH=$PATH:/home/robast/.local/bin' >> /home/robast/.bashrc
pip install -r /tmp/requirements.txt
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y