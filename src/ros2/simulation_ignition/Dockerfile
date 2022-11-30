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
ENV GAZ_IGN_RELEASE=garden

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-${GAZ_IGN_RELEASE}

# Must compile ros_gz from source because Humble + Garden is not an official combination
RUN mkdir ros_gz && cd ros_gz && git clone -b ros2 https://github.com/gazebosim/ros_gz.git \
    && apt update \
    && apt upgrade -y \
    && . /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update \
    && rosdep install --from-paths /ros_gz --ignore-src -r -y \
    && colcon build --build-base /workspace/build --install-base /workspace/install

RUN apt-get update && apt-get install -y \
    cmake \
    gdb \
    pylint \ 
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#COPY "/entrypoint.sh" "/entrypoint.sh"
#ENTRYPOINT ["/entrypoint.sh"]
