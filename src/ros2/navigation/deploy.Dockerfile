FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces_deploy  AS communication_interfaces


FROM ros:humble-ros-core as build
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu gnupg2 && \
    rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER ubuntu
ARG DEBIAN_FRONTEND=

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y \
  cmake \
  gdb \
  pylint \ 
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  #groot qt
  qtbase5-dev \
  libqt5svg5-dev \
  libzmq3-dev \
  libdw-dev \
  wget \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"
  # General ros stuff

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
  && curl http://repo.ros2.org/repos.key | apt-key add - \
  && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
  && apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir tmps && cd tmps && wget https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v8.2.0.tar.gz \
  && tar -xvzf v8.2.0.tar.gz \
  && cd openvdb-8.2.0/ \
  && mkdir builds && cd builds && cmake .. \
  && make && make install

RUN apt-get update && apt-get install -y \
  # Install ros distro testing packages
  # ros-${ROS_DISTRO}-ament-lint \
  # ros-${ROS_DISTRO}-launch-testing \
  # ros-${ROS_DISTRO}-launch-testing-ament-cmake \
  # ros-${ROS_DISTRO}-launch-testing-ros \
  python3-autopep8 \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-diagnostic-updater \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"
  
RUN apt-get update \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo 'export PS1="\[\e[31m\][\[\e[m\]\[\e[38;5;172m\]\u\[\e[m\]@\[\e[38;5;153m\]\h\[\e[m\]\[\e[38;5;214m\]\W\[\e[m\]\[\e[31m\]]\[\e[m\]\\$"' >> /home/$USER/.bashrc

RUN apt-get update && apt-get upgrade -y

COPY "." "/workspace/src/navigation" 
COPY --from=communication_interfaces /communication_interfaces workspace/src/communication_interfaces

WORKDIR /workspace
RUN rosdep update;\
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source /opt/ros/humble/setup.bash; \
    colcon build --cmake-args --packages-skip aws_hospital_world robast_map_update_module
COPY ./environment_vars.yaml /workspace/environment_vars.yaml


FROM ros:humble-ros-core as final 
COPY --from=build /workspace/install /workspace/install 
COPY --from=build /workspace/src/navigation/environment_vars.yaml /workspace/environment_vars.yaml

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2\ 
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-requests 

SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source install/setup.bash; \
    ros2 launch nav_bringup slam_toolbox_launch.py&

RUN cd /workspace; \
    source install/setup.bash; \
    ros2 launch nav_bringup nav_without_localization_launch.py&
    
CMD tail -f /dev/null
