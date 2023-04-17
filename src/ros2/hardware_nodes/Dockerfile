FROM ros:humble-ros-core

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

RUN echo "source install/setup.bash" >> /home/$USER/.bashrc


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


ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
