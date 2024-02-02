FROM moveit/moveit2:humble-release
ARG MOVEIT2_TUTORIALS_VERSION=humble

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    wget \	 
    gnupg \	  
    git \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo ignition 
ENV GZ_VERSION=fortress
ENV IGNITION_VERSION=fortress

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-${GZ_VERSION} \
    && apt-get update

# Nav part of installs
RUN apt-get update && apt-get install -y \
    cmake \
    gdb \
    pylint \ 
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    libboost-all-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

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




# Adding moveit2_tutorials as separate workspace which can be used for learning and testing purposes
# Can be removed once there is enough moveit2 skill and knowledge archieved
# TODO: Currently we use the rolling branch of moveit2, because we need iron (an there is no separate iron branch)
# TODO: We should regularly check if there is a dedicated branch for the ros version we are using
RUN mkdir -p workspace_moveit2/src && cd workspace_moveit2/src && git clone https://github.com/ros-planning/moveit2_tutorials -b ${MOVEIT2_TUTORIALS_VERSION} --depth 1 \
  && vcs import < moveit2_tutorials/moveit2_tutorials.repos \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_moveit2/src --ignore-src -r -y \
  && colcon build --parallel-workers 2 --build-base /workspace_moveit2/build --install-base /workspace_moveit2/install \
  && echo "if [ -f /workspace_moveit2/install/setup.bash ]; then source /workspace_moveit2/install/setup.bash; fi" >> /home/$USER/.bashrc

WORKDIR /robotnik
RUN git clone -b humble-devel https://github.com/RobotnikAutomation/robotnik_msgs.git && \
. /opt/ros/${ROS_DISTRO}/setup.sh && \
colcon build --packages-select robotnik_msgs --install-base /robast/${ROS_DISTRO}/ --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && echo "if [ -f /robast/humble/setup.bash ]; then source /robast/humble/setup.bash; fi" >> /home/$USER/.bashrc

# WORKDIR /openvdb
# # openvdb install for stvl (nav2)
# RUN mkdir tmps && cd tmps && wget https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v8.2.0.tar.gz \
#   && tar -xvzf v8.2.0.tar.gz \
#   && cd openvdb-8.2.0/ \
#   && mkdir builds && cd builds && cmake .. \
#   && make && make install

WORKDIR /workspace
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/src/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/src/dds_configs/cyclone_DDS_config.xml
