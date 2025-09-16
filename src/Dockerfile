# Dockerfile to build a core robast image with our custom interfaces, dds configs and error handling

ARG ROS_BUILD_DISTRO=humble
FROM ros:${ROS_BUILD_DISTRO}-ros-core AS robast-dev-core
ARG ROS_BUILD_DISTRO

# Common setup
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq bash-completion build-essential curl gosu locales lsb-release sudo tzdata vim wget && \
    rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER=robast
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=23
ENV ROS_DISTRO=${ROS_BUILD_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
  cmake \
  gdb \
  git \
  pylint \
  python3-argcomplete \
  python3-autopep8 \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-venv \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

WORKDIR /tmp_src
COPY ros2/communication_interfaces/ /tmp_src/communication_interfaces/
COPY ros2/error_handling/ /tmp_src/error_handling/
RUN git clone -b ros2-devel https://github.com/RobotnikAutomation/robotnik_msgs.git \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths /tmp_src/ --ignore-src -r -y \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
        --install-base /robast/${ROS_DISTRO}/ \
        --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF' \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc \
    && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp_src

COPY ros2/dds_configs/ /dds_configs/
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/dds_configs/cyclone_DDS_config.xml

USER robast
WORKDIR /workspace

#---------------------------------------------------------------------------------------------------#
# Dockerfile to build a base robast image with our custom interfaces, dds configs and error handling
ARG ROS_BUILD_DISTRO=humble
FROM ros:${ROS_BUILD_DISTRO}-ros-base AS robast-dev-base
ARG ROS_BUILD_DISTRO

ENV ROS_DISTRO=${ROS_BUILD_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    gdb \
    git \
    iputils-ping \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    vim && \
    rosdep init || echo "rosdep already initialized" && \
    rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER=robast
ARG DEBIAN_FRONTEND=

WORKDIR /tmp_src
COPY ros2/communication_interfaces/ /tmp_src/communication_interfaces/
COPY ros2/error_handling/ /tmp_src/error_handling/
RUN git clone -b ros2-devel https://github.com/RobotnikAutomation/robotnik_msgs.git \
    && rosdep update \
    && rosdep install --from-paths /tmp_src/ --ignore-src -r -y \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
        --install-base /robast/${ROS_DISTRO}/ \
        --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF' \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc \
    && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc \
    && rm -rf /tmp_src

COPY ros2/dds_configs/ /dds_configs/
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/dds_configs/cyclone_DDS_config.xml

USER robast
WORKDIR /workspace

#---------------------------------------------------------------------------------------------------#

# Dockerfile that contains the code of the Monorepo in the main branch.
# Should be used in the CI/CD pipeline to build the docker image.
ARG ROS_BUILD_DISTRO=humble
ARG BRANCH_TAG
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO}${BRANCH_TAG} AS robast-source

COPY Hardware/ /monorepo/src/Hardware/
COPY robot_backend/ /monorepo/src/robot_backend/
COPY ros2/utils/qos_bridge /monorepo/src/ros2/qos_bridge/
WORKDIR /monorepo
