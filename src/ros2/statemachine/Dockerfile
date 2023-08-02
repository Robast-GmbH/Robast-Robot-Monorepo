FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces  AS communication_interfaces
FROM ghcr.io/robast-gmbh/monorepo/error_handling  AS error_utils

FROM ros:humble-ros-base-jammy as base

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

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml

FROM ros:humble-ros-base-jammy as build
# hier will ich jetzt den code bauen aka copy + build in ros directory
COPY --from=base / /
COPY --from=communication_interfaces /communication_interfaces ws/src/communication_interfaces
COPY --from=error_utils /error_utils ws/src/error_utils
COPY . /ws/src/statemachine

WORKDIR /ws

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /opt/ros/${ROS_DISTRO}/


FROM ros:humble-ros-base-jammy as debug
# Stage debug
# hier will ich jetzt den code haben, rviz und ros2bag installen

FROM ros:humble-ros-base-jammy as production
# hier muss jetzt der gebaute code sein und ein skript o.Ä. um den code auch direkt auszuführen.
