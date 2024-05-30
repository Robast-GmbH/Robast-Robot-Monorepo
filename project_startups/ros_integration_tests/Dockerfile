FROM ghcr.io/robast-gmbh/monorepo/moveit:devel-humble


# Install Gazebo ignition 
ENV GZ_VERSION=fortress
ENV IGNITION_VERSION=fortress

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-${GZ_VERSION} \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# additional installs for the statemachine 
RUN apt-get update && apt-get install -y \
    build-essential \
    libzmq3-dev \
    libncurses5-dev \
    libncursesw5-dev \
    qtbase5-dev \
    libqt5svg5-dev \
    gdb \
    libpqxx-dev \
    lcov \
    && rm -rf /var/lib/apt/lists/*
    
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

# install to use the nvidia gpu in the docker
RUN apt-get update && apt-get install -y \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

WORKDIR /robotnik
RUN git clone -b humble-devel https://github.com/RobotnikAutomation/robotnik_msgs.git && \
. /opt/ros/${ROS_DISTRO}/setup.sh && \
colcon build --packages-select robotnik_msgs --install-base /robast/${ROS_DISTRO}/ --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && echo "if [ -f /robast/humble/setup.bash ]; then source /robast/humble/setup.bash; fi" >> /home/${USER}/.bashrc

WORKDIR /workspace
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/src/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/src/dds_configs/cyclone_DDS_config.xml
USER robast
