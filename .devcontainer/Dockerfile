FROM ubuntu:focal

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl \
    git \
    build-essential \
    vim \
    sudo \
    lsb-release \
    locales \
    bash-completion \
    tzdata \
    gosu && \
    rm -rf /var/lib/apt/lists/*
ARG DEBIAN_FRONTEND=
	
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER ubuntu
ENV USERNAME ubuntu

ENV ROS_DISTRO=foxy
ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib
ENV PATH=/opt/ros/foxy/bin:$PATH
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2


# Install ROS2
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
  && curl http://repo.ros2.org/repos.key | apt-key add - \
  && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-${ROS_DISTRO}.list' \
  && apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    python3-argcomplete \
    cmake \
    gdb \
    pylint3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    wget \
    # Install ros distro testing packages
    ros-${ROS_DISTRO}-ament-lint \
    ros-${ROS_DISTRO}-launch-testing \
    ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS_DISTRO}-launch-testing-ros \
    python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"


RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
RUN pip3 install connexion[swagger-ui]

#ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
#RUN apt-get update && apt-get install -y \
#  ros-foxy-desktop \
#  && rm -rf /var/lib/apt/lists/*
#ENV DEBIAN_FRONTEND=

  
#nav2 stuff
RUN apt-get update && apt-get install -y ros-foxy-navigation2 

#Turtlebot spezifisch
#RUN apt-get install -y ros-foxy-turtlebot3

#restapi
ENTRYPOINT ["/entrypoint.sh"]