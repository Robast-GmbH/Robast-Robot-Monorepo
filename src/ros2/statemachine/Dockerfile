ARG ROS_BUILD_DISTRO=humble
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO} AS devel
ARG ROS_BUILD_DISTRO

USER root

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    libzmq3-dev \
    libboost-dev \
    libncurses5-dev \
    libncursesw5-dev \
    qtbase5-dev \
    libqt5svg5-dev \
    gdb \
    && rosdep init || echo "rosdep already initialized"

WORKDIR /workspace

USER robast

RUN echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USER/.bashrc

############################## Build ##################################
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO} AS build
ARG ROS_BUILD_DISTRO

USER root

WORKDIR /ws
COPY . /ws/src/statemachine
RUN mkdir -p /robast/src/BT && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths /ws/src/ --ignore-src -r -y \
  . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
  . /robast/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

USER robast

############################## Release ##################################
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO} AS release
ARG ROS_BUILD_DISTRO

USER root

COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    libzmq3-dev

RUN echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc

USER robast
