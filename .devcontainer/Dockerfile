#FROM dorowu/ubuntu-desktop-lxde-vnc:focal
FROM ubuntu:focal
 
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
  dos2unix \
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
  # Dependencies needed to build the robot_localization package:
	libgeographic-dev \
  ros-${ROS_DISTRO}-geographic-msgs \
  ros-${ROS_DISTRO}-diagnostic-updater \
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
  && echo "cd ~/../../workspaces/Robast_RosTheron" >> /home/$USER/.bashrc

# Install Turtlebot3 
#RUN sudo apt install python3-vcstool \
#  && mkdir -p ~/turtlebot3_ws/src \
#  && cd ~/turtlebot3_ws \
#  && wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/foxy-devel/turtlebot3.repos \
#  && vcs import src < turtlebot3.repos \
#  && colcon build --symlink-install
#  && echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
ENTRYPOINT ["/entrypoint.sh"]

