FROM ros:humble as base


 
ARG DEBIAN_FRONTEND=noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER ubuntu
ARG DEBIAN_FRONTEND=

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2


# Install ROS2
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
  && curl http://repo.ros2.org/repos.key | apt-key add - \
  && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
  && apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-gazebo* \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*



RUN apt-get update && apt-get install -y \
  cmake \
  gdb \
  git \
  # pylint3 \ #gibts noch nicht 13.06.22
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
  #dos2unix \
  wget \
  && pip3 install tornado \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"
  # General ros stuff

  RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-xacro \
  # Install ros distro testing packages
  ros-${ROS_DISTRO}-ament-lint \
  ros-${ROS_DISTRO}-launch-testing \
  ros-${ROS_DISTRO}-launch-testing-ament-cmake \
  ros-${ROS_DISTRO}-launch-testing-ros \
  python3-autopep8 \
  # Install ROS Nav stuff 
  ros-${ROS_DISTRO}-cartographer \
  ros-${ROS_DISTRO}-cartographer-ros \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-robot-localization \
  # Depencencies needed to build the spatio_temporal_voxel_layer package:
  libopenvdb-dev \
  # Dependencies needed to build the robot_localization package:
	libgeographic-dev \
  ros-${ROS_DISTRO}-geographic-msgs \
  ros-${ROS_DISTRO}-diagnostic-updater \
  && pip3 install tornado \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"
  
RUN apt-get update \
  #&& apt-get install -y sudo \
  #&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  #&& chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  #&& rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "cd ~/../../workspaces/Robast_RosTheron" >> /home/$USER/.bashrc \
  && echo 'export PS1="\[\e[31m\][\[\e[m\]\[\e[38;5;172m\]\u\[\e[m\]@\[\e[38;5;153m\]\h\[\e[m\]\[\e[38;5;214m\]\W\[\e[m\]\[\e[31m\]]\[\e[m\]\\$"' >> /home/$USER/.bashrc

COPY "/entrypoint.sh" "/entrypoint.sh"
ENTRYPOINT ["/entrypoint.sh"]



#watch -d -n 0.5 nvidia-smi