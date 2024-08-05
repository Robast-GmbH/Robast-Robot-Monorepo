FROM nvcr.io/nvidia/l4t-base:r36.2.0
ARG GIT_TOKEN

ENV ROS_DISTRO=humble

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y python3-pip sudo locales software-properties-common curl git \
    && pip3 install Jetson.GPIO \
    && useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast \
    && echo robast:robast | chpasswd \
    && echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && usermod -aG sudo robast \
    && groupadd -f -r gpio \
    && usermod -a -G gpio robast \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get upgrade -y \
    && apt-get install -y ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rmw-cyclonedds-cpp python3-colcon-common-extensions python3-rosdep \
    && rosdep init || echo "rosdep already initialized" \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
ARG DEBIAN_FRONTEND

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/robast/.bashrc && \
    echo "if [ -f /robast/${ROS_DISTRO}/install/setup.bash ]; then source /robast/${ROS_DISTRO}/install/setup.bash; fi" >> /home/robast/.bashrc

ENV LANG=en_US.UTF-8
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml

COPY . /ws/src/nvidia_hardware_nodes/disinfection_module

WORKDIR /tmp
RUN git clone -b main https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git 

WORKDIR /ws
RUN mkdir -p /ws/src/ros2/ && mkdir /robast \
    && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs \
    && rosdep update \
    && rosdep install --from-paths /ws/src --ignore-src -r -y \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --install-base /robast/${ROS_DISTRO}/

USER robast
