#!/bin/bash
cd ~/../../workspaces/Robast_RosServer
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths src --ignore-src -r -y
#sudo dos2unix shell_setup.sh
#sudo /startup.sh

# Source
#source shell_setup.sh
