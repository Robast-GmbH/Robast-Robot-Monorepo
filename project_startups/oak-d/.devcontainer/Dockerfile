FROM ros:humble-ros-base-jammy

RUN apt update && apt upgrade -y && apt install -y \
    ros-${ROS_DISTRO}-depthai-ros-driver


ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml