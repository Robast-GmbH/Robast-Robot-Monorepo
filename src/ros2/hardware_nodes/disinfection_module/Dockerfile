ARG ROS_VERSION=humble

# First stage: Use the ROS base image to install ROS and other dependencies
FROM ros:${ROS_VERSION}-ros-core AS ros-base

# Second stage: Use the NVIDIA base image
FROM nvcr.io/nvidia/l4t-base:r32.4.3 AS final

COPY --from=ros-base /opt/ros /opt/ros

ENV ROS_DISTRO=${ROS_VERSION}

RUN apt-get update 
RUN apt-get install -y python3-pip
RUN pip3 install Jetson.GPIO

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
  echo robast:robast | chpasswd && \
  echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
