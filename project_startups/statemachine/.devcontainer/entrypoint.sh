#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y

# i dont want to accept that the vcpkg manage ris not working
# cd /opt/vcpkg/
# sudo chmod a+rw /opt/vcpkg/
# mkdir ~/.vcpkg
# touch ~/.vcpkg/vcpkg.path.txt
# ./bootstrap-vcpkg.sh && ./vcpkg integrate install && ./vcpkg integrate bash && echo 'export PATH=$PATH:/opt/vcpkg' >>~/.bashrc
