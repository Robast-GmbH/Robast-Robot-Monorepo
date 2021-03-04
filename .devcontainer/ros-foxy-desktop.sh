#!/bin/bash
set -eu

CHOOSE_ROS_DISTRO=foxy
INSTALL_PACKAGE=desktop

sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release


set +u

#source /opt/ros/${CHOOSE_ROS_DISTRO}/setup.bash

echo "success installing ROS2 ${CHOOSE_ROS_DISTRO}"
echo "Run 'source /opt/ros/${CHOOSE_ROS_DISTRO}/setup.bash'"

