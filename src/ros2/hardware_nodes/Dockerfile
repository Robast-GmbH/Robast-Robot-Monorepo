FROM ros:humble-ros-base-jammy as devel

ARG DEBIAN_FRONTEND=noninteractive
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
  echo robast:robast | chpasswd && \
  echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
ARG DEBIAN_FRONTEND=

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

RUN echo "source install/setup.bash" >> /home/$USER/.bashrc


RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  libboost-all-dev \
  ros-humble-ros-testing \
  ros-humble-ament-cmake-test \
  ros-humble-launch-testing \
  libpqxx-dev \
  git \
  && rosdep init || echo "rosdep already initialized"

RUN apt install ros-${ROS_DISTRO}-ros2-socketcan -y

#TODO: @unknown should web stff be realy part of this project?
RUN pip3 install fastapi -U \ 
  httpx 

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml


############################## Build ##################################
FROM ros:humble-ros-base-jammy as build
ARG GIT_TOKEN
COPY --from=devel / /
COPY . /ws/src/hardware_nodes
RUN apt-get update && apt-get install -y git


# TODO: No idea why but it seems that this version is broken: ros-humble-ros2-socketcan/jammy 1.3.0-1jammy.20240729.003002 amd64
# Therefore we need to install it manually for now
WORKDIR /ros2_socketcan_ws
RUN git clone -b ${ROS_DISTRO} https://github.com/autowarefoundation/ros2_socketcan.git
RUN rosdep update && \
  rosdep install --from-paths /ros2_socketcan_ws/ros2_socketcan --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/


WORKDIR /tmp
# TODO: Put in correct branch (main)
RUN git clone -b dev/RE-2121-fixed https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git 


WORKDIR /ws
RUN mkdir -p /ws/src/ros2/ && mkdir /robast\
  && mv /tmp/Monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /tmp/Monorepo/src/ros2/error_handling /ws/src/ros2/error_handling \
  && mv /tmp/Monorepo/libs ws/ \
  && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/

############################## Release ##################################
FROM ros:humble-ros-base-jammy as release

COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros2-socketcan \
    libboost-all-dev

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/robast/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

RUN apt-get update \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc