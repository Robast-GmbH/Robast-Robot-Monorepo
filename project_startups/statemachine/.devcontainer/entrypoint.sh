#!/bin/bash
#mkdir -p ~/rviz2_ws/src
#cd ~/rviz2_ws/src
#git clone https://github.com/ros2/rviz.git
#cp -r ./resource_retriever ~/rviz2_ws/src/rviz/rviz_rendering/include/
#colcon build --merge-instal
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -r -y

cd /opt/vcpkg/
sudo chmod a+rw /opt/vcpkg/
mkdir ~/.vcpkg
touch ~/.vcpkg/vcpkg.path.txt
./bootstrap-vcpkg.sh && ./vcpkg integrate install && ./vcpkg integrate bash && echo 'export PATH=$PATH:/opt/vcpkg' >>~/.bashrc

#sudo dos2unix shell_setup.sh
#sudo /startup.sh
