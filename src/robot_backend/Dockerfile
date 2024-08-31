ARG ROS_BUILD_DISTRO=humble

FROM ghcr.io/robast-gmbh/monorepo/building-docker:${ROS_BUILD_DISTRO} as build
ARG ROS_BUILD_DISTRO



FROM ros:${ROS_BUILD_DISTRO}-ros-core
ARG ROS_BUILD_DISTRO

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
  apt-get upgrade -yq && \
  apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
  rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
  echo robast:robast | chpasswd && \
  echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
ARG DEBIAN_FRONTEND=

ENV PATH=/opt/ros/${ROS_BUILD_DISTRO}/bin:$PATH
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=23


RUN apt-get update && apt-get install -y \
  cmake \
  gdb \
  git \
  iputils-ping \
  pylint \ 
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-venv \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"


RUN apt-get update && apt-get install -y \
  # Install ros distro testing packages
  python3-autopep8 \
  ros-${ROS_BUILD_DISTRO}-diagnostic-updater \
  ros-${ROS_BUILD_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_BUILD_DISTRO}-rosbridge-server \
  && rm -rf /var/lib/apt/lists/*

#TODO @TAlscher: this is just temporary to get this docker do be deployable
WORKDIR /release
COPY --from=build /monorepo/src/ros2/communication_interfaces /release/src/ros2/communication_interfaces
COPY --from=build /monorepo/src/ros2/dds_configs /dds_configs
COPY --from=build /monorepo/src/robot_backend /release/src/robot_backend

RUN pip install -r src/robot_backend/requirements.txt && \
  rosdep update && \
  rosdep install --from-paths /release/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /workspace
USER robast

RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USER}/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_BUILD_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_BUILD_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/${USER}/.bashrc

ENV CYCLONEDDS_URI=/dds_configs/cyclone_DDS_config.xml