ARG ROS_BUILD_DISTRO=jazzy
FROM ros:${ROS_BUILD_DISTRO}-perception AS devel

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
apt-get upgrade -yq && \
apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu gnupg2 && \
rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
ARG DEBIAN_FRONTEND=

RUN echo "ubuntu:ubuntu" | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


ENV USER=ubuntu
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

RUN apt-get update && apt-get install -y \
  cmake \
  gdb \
  pylint \ 
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  libusb-1.0-0-dev\
  wget \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"
  # General ros stuff

RUN apt-get update && apt-get install -y \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  # Install ros distro testing packages
  ros-${ROS_DISTRO}-ament-lint \
  ros-${ROS_DISTRO}-launch-testing \
  ros-${ROS_DISTRO}-launch-testing-ament-cmake \
  ros-${ROS_DISTRO}-launch-testing-ros \
  python3-autopep8 \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-foxglove-bridge \
  && rm -rf /var/lib/apt/lists/*
  
RUN echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo 'export PS1="\[\e[31m\][\[\e[m\]\[\e[38;5;172m\]\u\[\e[m\]@\[\e[38;5;153m\]\h\[\e[m\]\[\e[38;5;214m\]\W\[\e[m\]\[\e[31m\]]\[\e[m\]\\$"' >> /home/$USER/.bashrc

RUN apt-get update && apt-get upgrade -y

# COPY "/entrypoint.sh" "/entrypoint.sh"
#ENTRYPOINT ["/entrypoint.sh"]
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml

# build nav from source
WORKDIR /nav_ws/src
RUN git clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git
WORKDIR /nav_ws
RUN rosdep update && rosdep install --from-paths src/spatio_temporal_voxel_layer/openvdb_vendor --ignore-src -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-skip spatio_temporal_voxel_layer --cmake-args '-DCMAKE_BUILD_TYPE=Release'

RUN echo "if [ -f /nav_ws/install/setup.bash ]; then source /nav_ws/install/setup.bash; fi" >> /home/$USER/.bashrc


WORKDIR /workspace
USER ubuntu

############################## Build ##################################
FROM ros:${ROS_BUILD_DISTRO}-ros-base AS build
ARG ROS_BUILD_DISTRO
ARG GIT_TOKEN
ARG GIT_BRANCH=main
COPY --from=devel / /
COPY . /ws/src/navigation
RUN apt-get update && apt-get install -y git

WORKDIR /tmp
RUN git clone -b $GIT_BRANCH https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git 


WORKDIR /ws
RUN mkdir -p /ws/src/ros2/; mkdir /robast; mv /tmp/Monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs



RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/ --packages-skip robast_map_update_module robast_nav_recoveries

############################## Release ##################################
FROM ros:${ROS_BUILD_DISTRO}-perception AS release
ARG ROS_BUILD_DISTRO
RUN apt-get update && apt-get install -y \
  cmake \
  wget \
  libzmq3-dev \
  libdw-dev \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
  vim \
  && rm -rf /var/lib/apt/lists/* 

COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

RUN apt-get update && apt-get upgrade -y\
    && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libzmq3-dev \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/robast/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

RUN apt-get update \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc

ENV is_simulation=False
ENV config_directory=config
ENV robot_name=rb_theron
ENV init_x=1.46
ENV init_y=-2.89
ENV init_z=0.0
ENV init_yaw=1.57
ENV prefix=''
