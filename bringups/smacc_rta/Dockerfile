FROM ros:galactic-ros-core

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  dirmngr \
  git \
  curl \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  && rosdep init || echo "rosdep already initialized"

RUN curl -s https://1bde9b1a2e969457e8926f37a0a94a7f22c43a4056da8037:@packagecloud.io/install/repositories/robosoft-ai/SMACC2_RTA-startup/script.deb.sh | sudo bash

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt -y install ros-galactic-smacc2-rta

ENV USER robast
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc && \
    echo "if [ -f /home/ros2_ws/install/setup.bash ]; then source /home/ros2_ws/install/setup.bash; fi" >> /home/$USER/.bashrc && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers