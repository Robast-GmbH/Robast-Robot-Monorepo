ARG ROS_BUILD_DISTRO=humble
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO} AS devel
ARG ROS_BUILD_DISTRO

RUN apt-get update && apt-get install -y \
  libboost-all-dev \
  libpqxx-dev \
  ros-${ROS_BUILD_DISTRO}-ament-cmake-test \
  ros-${ROS_BUILD_DISTRO}-ros2-socketcan \
  ros-humble-ros-testing \
  alsa-utils \
  mpg123 \
  ros-${ROS_BUILD_DISTRO}-ros-testing \
  ros-${ROS_BUILD_DISTRO}-launch-testing \
  && rm -rf /var/cache/apk/* \
  && rosdep init || echo "rosdep already initialized"


############################## Build ##################################
FROM ghcr.io/robast-gmbh/monorepo/robast-source:${ROS_BUILD_DISTRO} AS build
ARG ROS_BUILD_DISTRO
COPY . /ws/src/hardware_nodes
USER root
RUN apt-get update && apt-get install -y git

WORKDIR /ws
RUN mkdir -p /ws/src/ros2/ && mkdir -p /robast \
  && mv /monorepo/src/ros2/qos_bridge /ws/src/ros2/qos_bridge \
  && mv /monorepo/src/Hardware/cpp/libs ws/

# TODO@Tobi: Running colcon build with -DBUILD_TESTING=OFF should be removed once build artifacts are used for tests with volume mounts
RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
  . /robast/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

############################## Release ##################################
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO} AS release
ARG ROS_BUILD_DISTRO
COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

USER root

RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y \
  libboost-all-dev \
  ros-${ROS_BUILD_DISTRO}-ros2-socketcan \
  alsa-utils \
  mpg123 \
  && rm -rf /var/cache/apk/*

ENV ROS_DOMAIN_ID=0

RUN echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc

USER robast
WORKDIR /robast
  
