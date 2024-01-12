#!/bin/bash
cd /workspace
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths src --ignore-src -r -y