ARG ROS_DISTRO=jazzy
#TODO@sagarvl96: define the correct image more specifically
FROM ros:${ROS_DISTRO} AS devel

RUN echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER=ubuntu

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    libgl1-mesa-glx\
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

WORKDIR /tmp
RUN git clone https://github.com/Robast-GmbH/LightGlue.git
# TODO@sagarvl96 fails in next line. bdist_wheel invalid command
RUN python3 LightGlue/setup.py bdist_wheel
RUN mkdir /ws
RUN if [ -f LightGlue/dist/*.whl ]; then mv LightGlue/dist/*.whl  /ws/ ; else echo "File not found!"; exit 1; fi
WORKDIR /ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml

USER ubuntu