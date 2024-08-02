FROM nvcr.io/nvidia/l4t-base:r36.2.0 as devel

ARG ROS_VERSION=humble

ENV ROS_DISTRO=${ROS_VERSION}

RUN apt-get update 
RUN apt-get install -y python3-pip sudo
RUN pip3 install Jetson.GPIO

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
  echo robast:robast | chpasswd && \
  echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER=robast

RUN usermod -aG sudo robast
RUN groupadd -f -r gpio
RUN usermod -a -G gpio robast

RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install -y software-properties-common && add-apt-repository universe

RUN apt update && apt install curl -y && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt upgrade -y

ARG DEBIAN_FRONTEND=noninteractive
RUN apt install -y ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rmw-cyclonedds-cpp python3-colcon-common-extensions

RUN apt-get update && apt-get install -y \
  python3-rosdep \
  && rosdep init || echo "rosdep already initialized"
ARG DEBIAN_FRONTEND  

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml

############################## Build ##################################
FROM ros:humble-ros-base-jammy as build
ARG GIT_TOKEN
ARG GIT_BRANCH=main
COPY --from=devel / /
COPY . /ws/src/hardware_nodes/disinfection_module
RUN apt-get update && apt-get install -y git


WORKDIR /tmp
RUN git clone -b $GIT_BRANCH https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git 


WORKDIR /ws
RUN mkdir -p /ws/src/ros2/ && mkdir /robast\
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
