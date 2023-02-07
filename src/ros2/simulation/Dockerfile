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

# Must compile ros_gz, gz-msgs and gz-transport from source because Humble + Garden is not an official combination
RUN mkdir -p workspace_ros_gz/src && cd workspace_ros_gz/src && git clone -b ros2 https://github.com/gazebosim/ros_gz.git \
  && cd /workspace_ros_gz/src && git clone -b gz-msgs9 https://github.com/gazebosim/gz-msgs.git \
  && cd /workspace_ros_gz/src && git clone -b gz-transport12 https://github.com/gazebosim/gz-transport.git \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_ros_gz/src --ignore-src -r -y \
  && colcon build --build-base /workspace_ros_gz/build --install-base /workspace_ros_gz/install \
  && echo "if [ -f /workspace_ros_gz/install/setup.bash ]; then source /workspace_ros_gz/install/setup.bash; fi" >> /home/$USER/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV IGN_FILE_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
