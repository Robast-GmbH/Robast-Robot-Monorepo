# Dockerfile that contains the code of the Monorepo in the main branch.
# Should be used in the CI/CD pipeline to build the docker image.
ARG ROS_BUILD_DISTRO=humble
FROM ros:${ROS_BUILD_DISTRO}-ros-base

COPY . /monorepo/src
WORKDIR /monorepo

RUN rm -rf  /monorepo/src/ros2/moveit \
            /monorepo/src/ros2/perception \
            /monorepo/src/ros2/simulation \
            /monorepo/src/ros2/test_and_debug \
            /monorepo/src/ros2/robot_description \
            /monorepo/src/ros2/simple_fleetmanagement

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq bash-completion build-essential curl git vim && \
    rm -rf /var/lib/apt/lists/*
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
