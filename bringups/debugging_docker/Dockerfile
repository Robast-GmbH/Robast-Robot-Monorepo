FROM ros:humble-ros-core

RUN apt update -y && apt upgrade -y && apt install -y \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-depthai-ros-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-lifecycle-msgs \
    ros-${ROS_DISTRO}-rosbridge-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-slam-toolbox

COPY recording_scripts recording_scripts
COPY nav2_default_view.rviz nav2_default_view.rviz

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu gnupg2 && \
    rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
ARG DEBIAN_FRONTEND=

#docker run --name=debug -d --restart unless-stopped --network=host -v ./:/workspace/scripts -v /media/robast/PortableSSD/hagen:/workspace/bags ghcr.io/robast-gmbh/monorepo/debug /bin/bash -c 'su robast; source /opt/ros/humble/setup.bash; ros2 launch foxglove_bridge foxglove_bridge_launch.xml'