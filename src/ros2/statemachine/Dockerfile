FROM ros:humble-ros-base-jammy

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libzmq3-dev \
    libboost-dev \
    libncurses5-dev \
    libncursesw5-dev \
    qtbase5-dev \
    libqt5svg5-dev \
    gdb \
    && rosdep init || echo "rosdep already initialized"

WORKDIR /workspace

RUN apt-get update \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USER/.bashrc

ENV ROS_DOMAIN_ID=44
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
