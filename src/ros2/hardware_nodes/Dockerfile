ARG ROS_BUILD_DISTRO=humble
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO} as devel
ARG ROS_BUILD_DISTRO

RUN apt-get update && apt-get install -y \
  libboost-all-dev \
  libpqxx-dev \
  ros-${ROS_BUILD_DISTRO}-ament-cmake-test \
  ros-${ROS_BUILD_DISTRO}-ros2-socketcan \
  ros-${ROS_BUILD_DISTRO}-ros-testing \
  ros-${ROS_BUILD_DISTRO}-launch-testing \
  && rm -rf /var/cache/apk/* \
  && rosdep init || echo "rosdep already initialized"


############################## Build ##################################
FROM ghcr.io/robast-gmbh/monorepo/robast-source:${ROS_BUILD_DISTRO} as build
ARG ROS_BUILD_DISTRO
COPY . /ws/src/hardware_nodes
USER root
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
  && mv /monorepo/src/ros2/qos_bridge /ws/src/ros2/qos_bridge \
  && mv /monorepo/src/Hardware/cpp/libs ws/

# TODO@Tobi: Running colcon build with -DBUILD_TESTING=OFF should be removed once build artifacts are used for tests with volume mounts
RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

############################## Release ##################################
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO} as release
ARG ROS_BUILD_DISTRO
COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

USER root

RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y \
  libboost-all-dev \
  ros-${ROS_BUILD_DISTRO}-ros2-socketcan \
  && rm -rf /var/cache/apk/*

ENV ROS_DOMAIN_ID=0

RUN echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc

USER robast
WORKDIR /robast
  
