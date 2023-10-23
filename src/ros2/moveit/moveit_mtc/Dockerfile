FROM ros:iron as devel

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
    
#update drivers to avoid a blackscreen #TODO @all regularly check if its fixed in base docker
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository ppa:kisak/kisak-mesa -y\
    && apt upgrade -y

# Install ros2_control from source as long as we use it for learning and testing purposes
RUN mkdir -p workspace_ros_control/src && cd workspace_ros_control/src \
  && git clone https://github.com/ros-controls/ros2_control -b master \
  && git clone https://github.com/ros-controls/ros2_controllers -b master \
  && git clone https://github.com/ros-controls/ros2_control_demos -b master \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_ros_control/src --ignore-src -r -y \
  && colcon build --parallel-workers 3 --build-base /workspace_ros_control/build --install-base /workspace_ros_control/install \
  && echo "if [ -f /workspace_ros_control/install/setup.bash ]; then source /workspace_ros_control/install/setup.bash; fi" >> /home/$USER/.bashrc

# Adding moveit2_tutorials as separate workspace which can be used for learning and testing purposes
# Can be removed once there is enough moveit2 skill and knowledge archieved
RUN mkdir -p workspace_moveit2/src && cd workspace_moveit2/src && git clone https://github.com/ros-planning/moveit2_tutorials -b main --depth 1 \
  && vcs import < moveit2_tutorials/moveit2_tutorials.repos \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_moveit2/src --ignore-src -r -y \
  && colcon build --parallel-workers 3 --build-base /workspace_moveit2/build --install-base /workspace_moveit2/install \
  && echo "if [ -f /workspace_moveit2/install/setup.bash ]; then source /workspace_moveit2/install/setup.bash; fi" >> /home/$USER/.bashrc

# Install PlotJuggler as debugging tool to view controller_state data and so forth
# TODO@Jacob: Remove this from the docker for production
RUN apt-get update && apt install ros-${ROS_DISTRO}-plotjuggler-ros -y

# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml
ENV IGN_FILE_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
