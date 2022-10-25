FROM ros:humble

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  dirmngr \
  git \
  curl \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-plansys2-* \
  ros-${ROS_DISTRO}-test-msgs \
  && rosdep init || echo "rosdep already initialized"

RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -q -y --no-install-recommends xterm

ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENV USER robast
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc && \
    echo "if [ -f /home/ros2_ws/install/setup.bash ]; then source /home/ros2_ws/install/setup.bash; fi" >> /home/$USER/.bashrc && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    ros-${ROS_DISTRO}-gazebo* \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*
