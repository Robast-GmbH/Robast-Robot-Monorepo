ARG ROS_BUILD_DISTRO=humble
FROM ros:${ROS_BUILD_DISTRO}-ros-base as devel
ARG ROS_BUILD_DISTRO

ARG DEBIAN_FRONTEND=noninteractive
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
  echo robast:robast | chpasswd && \
  echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
ARG DEBIAN_FRONTEND=

RUN echo "if [ -f /opt/ros/${ROS_BUILD_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_BUILD_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

RUN echo "source install/setup.bash" >> /home/$USER/.bashrc


RUN apt-get update && apt-get install -y \
  build-essential \
  git \
  libboost-all-dev \
  libpqxx-dev \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-humble-ament-cmake-test \
  ros-${ROS_BUILD_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_BUILD_DISTRO}-ros2-socketcan \
  ros-humble-ros-testing \
  ros-humble-launch-testing \
  && rm -rf /var/cache/apk/* \
  && rosdep init || echo "rosdep already initialized"

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml


############################## Build ##################################
FROM ghcr.io/robast-gmbh/monorepo/building-docker:${ROS_BUILD_DISTRO} as build
ARG ROS_BUILD_DISTRO
COPY . /ws/src/hardware_nodes
RUN apt-get update && apt-get install -y git


# TODO: No idea why but it seems that this version is broken: ros-humble-ros2-socketcan/jammy 1.3.0-1jammy.20240729.003002 amd64
# Therefore we need to install it manually for now
WORKDIR /ros2_socketcan_ws
RUN git clone -b ${ROS_BUILD_DISTRO} https://github.com/autowarefoundation/ros2_socketcan.git
RUN rosdep update && \
  rosdep install --from-paths /ros2_socketcan_ws/ros2_socketcan --ignore-src -r -y && \
  . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

WORKDIR /ws
RUN mkdir -p /ws/src/ros2/ && mkdir -p /robast \
  && mv /monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /monorepo/src/ros2/error_handling /ws/src/ros2/error_handling \
  && mv /monorepo/src/ros2/utils/qos_bridge /ws/src/ros2/qos_bridge \
  && mv /monorepo/src/Hardware/cpp/libs ws/ \
  && mv /monorepo/src/ros2/dds_configs /robast/dds_configs

# TODO@Tobi: Running colcon build with -DBUILD_TESTING=OFF should be removed once build artifacts are used for tests with volume mounts
RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

############################## Release ##################################
FROM ros:${ROS_BUILD_DISTRO}-ros-base as release
ARG ROS_BUILD_DISTRO
COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y \
  libboost-all-dev \
  ros-${ROS_BUILD_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_BUILD_DISTRO}-ros2-socketcan \
  && rm -rf /var/cache/apk/*

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/robast/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

RUN apt-get update \
  && echo "if [ -f /robast/${ROS_BUILD_DISTRO}/setup.bash ]; then source /robast/${ROS_BUILD_DISTRO}/setup.bash; fi" >> ~root/.bashrc