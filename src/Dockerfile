# Dockerfile that contains the code of the Monorepo in the main branch.
# Should be used in the CI/CD pipeline to build the docker image.
ARG ROS_BUILD_DISTRO=humble
FROM ros:${ROS_BUILD_DISTRO}-ros-base

WORKDIR /monorepo
COPY . /monorepo/src

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq bash-completion build-essential curl git gnupg2 gosu locales lsb-release sudo tzdata vim wget && \
    rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER=ubuntu
ARG DEBIAN_FRONTEND=


RUN apt-get update && apt-get install -y \
cmake \
gdb \
pylint \ 
python3-argcomplete \
python3-colcon-common-extensions \
python3-pip \
python3-rosdep \
&& rm -rf /var/lib/apt/lists/* \
&& rosdep init || echo "rosdep already initialized"

RUN apt-get update && rosdep update && \
rosdep install --from-paths /monorepo/src/ros2 --ignore-src -r -y
