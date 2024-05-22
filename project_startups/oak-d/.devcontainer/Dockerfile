FROM ros:humble-ros-base-jammy

RUN apt update && apt upgrade -y && apt install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-depthai-ros-driver

ENV ROS_DOMAIN_ID=23
# dds setup / configuration
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml