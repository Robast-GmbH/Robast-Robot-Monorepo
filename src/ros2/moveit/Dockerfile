ARG ROS_VERSION=jazzy
FROM ros:${ROS_VERSION} AS devel
ARG ROS_CONTROL_VERSION=master
ARG MOVEIT2_TUTORIALS_VERSION=main

RUN echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV USER=ubuntu
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc

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

# Install ros2_control from source as long as we use it for learning and testing purposes
# TODO: Currently we use the rolling branch of ros2_control, because we need iron (an there is no separate iron branch)
# TODO: We should regularly check if there is a dedicated branch for the ros version we are using
RUN mkdir -p workspace_ros_control/src && cd workspace_ros_control/src \
  && git clone https://github.com/ros-controls/ros2_control -b ${ROS_DISTRO} \
  && git clone https://github.com/ros-controls/ros2_controllers -b ${ROS_DISTRO} \
  && vcs import < ros2_control/ros2_control.${ROS_DISTRO}.repos \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_ros_control/src --ignore-src -r -y \
  && colcon build --parallel-workers 1 --build-base /workspace_ros_control/build --install-base /workspace_ros_control/install \
  && echo "if [ -f /workspace_ros_control/install/setup.bash ]; then source /workspace_ros_control/install/setup.bash; fi" >> /home/${USER}/.bashrc

# Adding moveit2_tutorials as separate workspace which can be used for learning and testing purposes
# Can be removed once there is enough moveit2 skill and knowledge archieved
# # TODO: Currently we use the rolling branch of moveit2, because we need iron (an there is no separate iron branch)
# # TODO: We should regularly check if there is a dedicated branch for the ros version we are using
# RUN mkdir -p workspace_moveit2/src && cd workspace_moveit2/src && git clone https://github.com/ros-planning/moveit2_tutorials -b ${MOVEIT2_TUTORIALS_VERSION} --depth 1 \
#   && vcs import < moveit2_tutorials/moveit2_tutorials.repos \
#   && apt update \
#   && apt upgrade -y \
#   && . /opt/ros/${ROS_DISTRO}/setup.sh \
#   && rosdep update \
#   && rosdep install --from-paths /workspace_moveit2/src --ignore-src -r -y \
#   && colcon build --parallel-workers 2 --build-base /workspace_moveit2/build --install-base /workspace_moveit2/install \
#   && echo "if [ -f /workspace_moveit2/install/setup.bash ]; then source /workspace_moveit2/install/setup.bash; fi" >> /home/${USER}/.bashrc

# Install moveit_task_constructor from source
RUN mkdir -p /workspace_moveit_task_constructor/src && cd /workspace_moveit_task_constructor/src \
  && git clone --recurse-submodules https://github.com/moveit/moveit_task_constructor.git -b ${ROS_DISTRO} \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_moveit_task_constructor/src --ignore-src -r -y \
  && colcon build --parallel-workers 1 --build-base /workspace_moveit_task_constructor/build --install-base /workspace_moveit_task_constructor/install \
  && echo "if [ -f /workspace_moveit_task_constructor/install/setup.bash ]; then source /workspace_moveit_task_constructor/install/setup.bash; fi" >> /home/${USER}/.bashrc

# Install PlotJuggler as debugging tool to view controller_state data and so forth
# TODO: Remove this once we left the early development phase
RUN sudo apt-get update && sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros -y

# Adds the workspace to the bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/${USER}/.bashrc

# Builds depthai-ros from source because it is not available as debian package for jazzy yet (as of 07.08.2024)
# TODO: Remove this once depthai-ros is available as debian package for jazzy
WORKDIR /depthai_ws/src
RUN git clone -b ${ROS_DISTRO} https://github.com/luxonis/depthai-ros.git
WORKDIR /depthai_ws
RUN rosdep update && rosdep install --from-paths src/depthai-ros --ignore-src -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
RUN echo "if [ -f /depthai_ws/install/setup.bash ]; then source /depthai_ws/install/setup.bash; fi" >> /home/${USER}/.bashrc


# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml
ENV IGN_FILE_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
