ARG ROS_BUILD_DISTRO=humble
FROM ros:humble-ros-base AS devel
ARG ROS_BUILD_DISTRO

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

WORKDIR /robotnik
RUN git clone -b ros2-devel https://github.com/RobotnikAutomation/robotnik_msgs.git && \
. /opt/ros/${ROS_DISTRO}/setup.sh && \
colcon build --packages-select robotnik_msgs robotnik_safety_msgs --install-base /robast/${ROS_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

WORKDIR /workspace

RUN apt-get update \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /robast/humble/setup.bash ]; then source /robast/humble/setup.bash; fi" >> /home/$USER/.bashrc 

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml

############################## Build ##################################
FROM ghcr.io/robast-gmbh/monorepo/building-docker:${ROS_BUILD_DISTRO} AS build
ARG ROS_BUILD_DISTRO

COPY --from=devel /robast /robast

WORKDIR /tmp
RUN git clone -b 4.5.x https://github.com/BehaviorTree/BehaviorTree.CPP.git

WORKDIR /ws
RUN mkdir -p /ws/src/ros2/ /robast/src/BT && \
  mv /tmp/BehaviorTree.CPP /robast/src/BT && \
  mv /monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces && \
  mv /monorepo/src/ros2/error_handling /ws/src/ros2/error_handling && \
  mv /monorepo/src/ros2/dds_configs /robast/dds_configs && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths /ws/src/ros2 --ignore-src -r -y \
  . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
  . /robast/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

FROM build AS statemachine-build

ARG ROS_BUILD_DISTRO
WORKDIR /ws
COPY . /ws/src/statemachine

RUN rosdep install --from-paths /ws/src/statemachine --ignore-src -r -y  &&\
  . /robast/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

############################## Release ##################################
FROM ros:${ROS_BUILD_DISTRO}-ros-base AS release
ARG ROS_BUILD_DISTRO

COPY --from=statemachine-build /opt/ros /opt/ros
COPY --from=statemachine-build /robast /robast

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libzmq3-dev

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/robast/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

RUN apt-get update \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc
