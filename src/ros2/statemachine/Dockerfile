FROM ros:jazzy-ros-base as devel

RUN echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER ubuntu

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libzmq3-dev \
    libboost-dev \
    libncurses5-dev \
    libncursesw5-dev \
    qtbase5-dev \
    libqt5svg5-dev \
    gdb \
    && rosdep init || echo "rosdep already initialized"

WORKDIR /robotnik
RUN git clone -b ros2-devel https://github.com/RobotnikAutomation/robotnik_msgs.git && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --packages-select robotnik_msgs --install-base /$USER/${ROS_DISTRO}/ --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /workspace

RUN apt-get update \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /$USER/${ROS_DISTRO}/setup.bash ]; then source /$USER/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc 

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml

USER ubuntu

############################## Build ##################################
FROM ros:humble-ros-base-jammy as build
ARG GIT_TOKEN
ARG GIT_BRANCH=main
COPY --from=devel / /
COPY . /ws/src/statemachine
RUN apt-get update && apt-get install -y git


WORKDIR /tmp
RUN git clone -b $GIT_BRANCH https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git 



WORKDIR /ws
RUN mkdir -p /ws/src/ros2/; mkdir /robast; \
  && mv /tmp/Monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /tmp/Monorepo/src/ros2/error_handling /ws/src/ros2/error_handling \
  && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  . /robast/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/

WORKDIR /robotnik

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/


############################## Release ##################################
FROM ros:humble-ros-base-jammy as release

COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

RUN echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER ubuntu

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libzmq3-dev

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/robast/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

RUN apt-get update \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc

USER ubuntu