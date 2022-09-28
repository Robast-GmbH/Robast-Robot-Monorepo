FROM ros:humble

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    ros-${ROS_DISTRO}-gazebo* \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    cmake \
    gdb \
    git \
    pylint \ 
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

#COPY "/entrypoint.sh" "/entrypoint.sh"
#ENTRYPOINT ["/entrypoint.sh"]