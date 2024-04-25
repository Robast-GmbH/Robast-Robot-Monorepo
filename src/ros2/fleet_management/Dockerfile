ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=7


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
  python3-venv \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

RUN pip3 install roslibpy pipenv nudged

RUN apt-get update && apt-get install -y \
  # Install ros distro testing packages
  python3-autopep8 \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-diagnostic-updater \
  && rm -rf /var/lib/apt/lists/*
  
WORKDIR /rmf-web-workspace
RUN git clone https://github.com/open-rmf/rmf-web.git
RUN chown -Rf robast .

WORKDIR /workspace
USER robast

RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USER}/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/${USER}/.bashrc


