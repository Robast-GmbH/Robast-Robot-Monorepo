FROM moveit/moveit2:humble-release

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
    && add-apt-repository ppa:kisak/kisak-mesa -y\
    && apt upgrade -y


# Must compile ros_gz, gz-msgs and gz-transport from source because Humble + Garden is not an official combination
RUN mkdir -p workspace_ros_gz/src && cd workspace_ros_gz/src && git clone -b humble https://github.com/gazebosim/ros_gz.git \
    && cd /workspace_ros_gz/src && git clone -b gz-msgs9 https://github.com/gazebosim/gz-msgs.git \
    && cd /workspace_ros_gz/src && git clone -b gz-transport12 https://github.com/gazebosim/gz-transport.git \
    && apt update \
    && apt upgrade -y \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install --from-paths /workspace_ros_gz/src --ignore-src -r -y \
    && colcon build --build-base /workspace_ros_gz/build --install-base /workspace_ros_gz/install \
    && echo "if [ -f /workspace_ros_gz/install/setup.bash ]; then source /workspace_ros_gz/install/setup.bash; fi" >> /home/$USER/.bashrc


#install RMF
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y\
    ros-${ROS_DISTRO}-rmf-api-msgs\
    ros-${ROS_DISTRO}-rmf-utils\
    ros-${ROS_DISTRO}-rmf-robot-sim-common\
    ros-${ROS_DISTRO}-rmf-robot-sim-gz-plugins\
    ros-${ROS_DISTRO}-rmf-scheduler-msgs \
    ros-${ROS_DISTRO}-rmf-battery \
    ros-${ROS_DISTRO}-rmf-building-map-msgs\
    ros-${ROS_DISTRO}-rmf-building-map-tools\
    ros-${ROS_DISTRO}-rmf-building-sim-gz-plugins\
    ros-${ROS_DISTRO}-rmf-task\
    ros-${ROS_DISTRO}-rmf-task-msgs\
    ros-${ROS_DISTRO}-rmf-task-ros2\
    ros-${ROS_DISTRO}-rmf-task-sequence\
    ros-${ROS_DISTRO}-rmf-fleet-adapter\
    ros-${ROS_DISTRO}-rmf-fleet-adapter-python\
    ros-${ROS_DISTRO}-rmf-fleet-msgs\
    ros-${ROS_DISTRO}-rmf-traffic-ros2\
    ros-${ROS_DISTRO}-rmf-traffic\
    ros-${ROS_DISTRO}-rmf-traffic-editor\
    ros-${ROS_DISTRO}-rmf-traffic-msgs\ 
    ros-${ROS_DISTRO}-rmf-visualization\
    ros-${ROS_DISTRO}-rmf-visualization-msgs\
    ros-${ROS_DISTRO}-rmf-visualization-fleet-states\
    ros-${ROS_DISTRO}-rmf-visualization-floorplans\
    ros-${ROS_DISTRO}-rmf-visualization-rviz2-plugins\
    ros-${ROS_DISTRO}-rmf-websocket

RUN python3 -m pip install -U  cyclonedds

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV IGN_FILE_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
