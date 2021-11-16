FROM dorowu/ubuntu-desktop-lxde-vnc:focal as base


 
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

ENV ROS_DISTRO=galactic
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
  pylint3 \
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
  # Depencencies needed to build the spatio_temporal_voxel_layer package:
  libopenvdb-dev \
  # Dependencies needed to build the robot_localization package:
	libgeographic-dev \
  ros-${ROS_DISTRO}-geographic-msgs \
  ros-${ROS_DISTRO}-diagnostic-updater \
  && pip3 install tornado \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"
  
  
#Relevent for Linux docker
# Install glvnd + opengl libraries
# They don't come over apt for Ubuntu 16.04, so we just grab them from another img
# COPY --from=nvidia/opengl:1.1-glvnd-runtime-ubuntu20.04 \
#   /usr/lib/x86_64-linux-gnu \
#   /usr/local/lib/x86_64-linux-gnu
# RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
#     ldconfig
# Set up env variables to enable nvidia for opengl
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:-all}
# ENV NVIDIA_REQUIRE_CUDA "cuda>=8.0"
# # Some QT-Apps/Gazebo don't show controls without this
# ENV QT_X11_NO_MITSHM 1

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

  # RUN apt-get update && apt-get install -y \
  #   libxext-dev \
  #   libxrender-dev \
  #   libxtst-dev \
  #   && rm -rf /var/lib/apt/lists/* 



  # ENV QT_DEBUG_PLUGINS=1
  # ENV QT_QPA_PLATFORM=xcb
  # ENV QT_QPA_PLATFORM_PLUGIN_PATH=/opt/Qt/${QT_VERSION}/gcc_64/plugins
  # ENV QT_PLUGIN_PATH=/opt/Qt/${QT_VERSION}/gcc_64/plugins
  # ENV DISPLAY=:1
# Install Turtlebot3 
#RUN sudo apt install python3-vcstool \
#  && mkdir -p ~/turtlebot3_ws/src \
#  && cd ~/turtlebot3_ws \
#  && wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/foxy-devel/turtlebot3.repos \
#  && vcs import src < turtlebot3.repos \
#  && colcon build --symlink-install
#  && echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc

COPY "/entrypoint.sh" "/entrypoint.sh"
ENTRYPOINT ["/entrypoint.sh"]



#watch -d -n 0.5 nvidia-smi