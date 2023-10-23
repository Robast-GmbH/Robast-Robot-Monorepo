FROM ros:iron as base

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
ENV GZ_VERSION=garden
ENV IGNITION_VERSION=garden

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-${GZ_VERSION} \
    && apt-get update

# Must compile ros_gz, gz-msgs and gz-transport from source because iron + Garden is not an official combination
RUN mkdir -p workspace_ros_gz/src && cd workspace_ros_gz/src && git clone -b iron https://github.com/gazebosim/ros_gz.git \
    && cd /workspace_ros_gz/src && git clone -b gz-msgs9 https://github.com/gazebosim/gz-msgs.git \
    && cd /workspace_ros_gz/src && git clone -b gz-transport12 https://github.com/gazebosim/gz-transport.git \
    && apt update \
    && apt upgrade -y \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install --from-paths /workspace_ros_gz/src --ignore-src -r -y \
    && colcon build --build-base /workspace_ros_gz/build --install-base /workspace_ros_gz/install \
    && echo "if [ -f /workspace_ros_gz/install/setup.bash ]; then source /workspace_ros_gz/install/setup.bash; fi" >> /home/$USER/.bashrc

# Must build gz_ros2_control from source because as of now (19.10.23) there are no binaries
# for the combination iron + garden according to: https://github.com/ros-controls/gz_ros2_control
RUN mkdir -p workspace_ros_control/src && cd workspace_ros_control/src \
  && git clone https://github.com/ros-controls/gz_ros2_control -b master \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_ros_control/src --ignore-src -r -y \
  && colcon build --parallel-workers 3 --build-base /workspace_ros_control/build --install-base /workspace_ros_control/install \
  && echo "if [ -f /workspace_ros_control/install/setup.bash ]; then source /workspace_ros_control/install/setup.bash; fi" >> /home/$USER/.bashrc

# Must compile moveit_task_constructor_msgs from source, because the ros package is not released yet (22.09.23)
# https://index.ros.org/p/moveit_task_constructor_msgs/
# TODO: Remove this once moveit_task_constructor_msgs is released as ros package 
RUN mkdir -p workspace_mtc/src && cd workspace_mtc/src && git init moveit_task_constructor \
    && cd /workspace_mtc/src/moveit_task_constructor && git remote add -f origin https://github.com/ros-planning/moveit_task_constructor.git \
    && git config core.sparseCheckout true \
    && echo "msgs" >> .git/info/sparse-checkout \
    && git pull origin ros2 \
    && apt update \
    && apt upgrade -y \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install --from-paths /workspace_mtc/src --ignore-src -r -y \
    && colcon build --build-base /workspace_mtc/build --install-base /workspace_mtc/install \
    && echo "if [ -f /workspace_mtc/install/setup.bash ]; then source /workspace_mtc/install/setup.bash; fi" >> /home/$USER/.bashrc

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
    
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

#instal to use the nvidia gpu in the docker
RUN apt-get update && apt-get install -y \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

#update drivers to avoid a blackscreen #TODO @all regularly check if its fixed in base docker
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository ppa:kisak/kisak-mesa -y \
    && apt upgrade -y

# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml
ENV IGN_FILE_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
