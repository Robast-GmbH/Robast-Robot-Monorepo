FROM ros:humble-ros-base

COPY . /workspace/src

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    build-essential \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

RUN apt-get update && apt-get upgrade -y && \
    rosdep update && \
    rosdep install --from-paths /workspace/src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    cd /workspace && \
    colcon build