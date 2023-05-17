FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  libboost-all-dev \
  ros-humble-ros-testing \
  ros-humble-ament-cmake-test \
  ros-humble-launch-testing \
  libpqxx-dev \
  git \
  && rosdep init || echo "rosdep already initialized"

RUN pip3 install fastapi -U \
  httpx 

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
