ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS devel 

RUN echo "ubuntu:ubuntu" | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

RUN echo "source install/setup.bash" >> /home/$USER/.bashrc


RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  libboost-all-dev \
  ros-${ROS_DISTRO}-ros-testing \
  ros-${ROS_DISTRO}-ament-cmake-test \
  ros-${ROS_DISTRO}-launch-testing \
  libpqxx-dev \
  gdb \
  git \
  && rosdep init || echo "rosdep already initialized"

RUN apt install ros-${ROS_DISTRO}-ros2-socketcan -y

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml

USER ubuntu


############################## Build ##################################
FROM ros:jazzy-ros-base AS build
ARG GIT_TOKEN
ARG GIT_BRANCH=main
COPY --from=devel / /
COPY . /ws/src/hardware_nodes
RUN apt-get update && apt-get install -y git


WORKDIR /tmp
RUN git clone -b $GIT_BRANCH https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git 


WORKDIR /ws
RUN mkdir -p /ws/src/ros2/ && mkdir /robast \
  && mv /tmp/Monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /tmp/Monorepo/src/ros2/error_handling /ws/src/ros2/error_handling \
  && mv /tmp/Monorepo/libs ws/ \
  && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/

############################## Release ##################################
FROM ros:jazzy-ros-base AS release

COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros2-socketcan \
    libboost-all-dev

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/robast/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

USER ubuntu