FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
  echo robast:robast | chpasswd && \
  echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
ARG DEBIAN_FRONTEND=

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

RUN echo "source install/setup.bash" >> /home/$USER/.bashrc


RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  libboost-all-dev \
  ros-humble-ros-testing \
  ros-humble-ament-cmake-test \
  ros-humble-launch-testing \
  libpqxx-dev \
  git \
  && rosdep init || echo "rosdep already initialized"

# Install ros2_control from source as long as we are atively working with it
RUN mkdir -p workspace_ros_controls/src && cd workspace_ros_controls/src \
  && git clone https://github.com/ros-controls/ros2_control -b ${ROS_DISTRO} \
  && git clone https://github.com/ros-controls/ros2_controllers -b ${ROS_DISTRO} \
  && git clone https://github.com/ros-controls/ros2_control_demos -b ${ROS_DISTRO} \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_ros_controls/src --ignore-src -r -y \
  && colcon build --parallel-workers 3 --build-base /workspace_ros_controls/build --install-base /workspace_ros_controls/install \
  && echo "if [ -f /workspace_ros_controls/install/setup.bash ]; then source /workspace_ros_controls/install/setup.bash; fi" >> /home/$USER/.bashrc

#update drivers to avoid a blackscreen #TODO @all regularly check if its fixed in base docker
RUN apt-get update && apt-get install -y \
  software-properties-common \
  && add-apt-repository ppa:kisak/kisak-mesa -y\
  && apt upgrade -y

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
