FROM ros:humble

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

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

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GAZEBO_MODEL_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
ENV GAZEBO_MODEL_PATH=/workspace/src/simulation/tiplu_world/models/furniture:${GAZEBO_MODEL_PATH}
ENV GAZEBO_PLUGIN_PATH=/workspace/install/gazebo_plugins:${GAZEBO_PLUGIN_PATH}

#COPY "/entrypoint.sh" "/entrypoint.sh"
#ENTRYPOINT ["/entrypoint.sh"]
