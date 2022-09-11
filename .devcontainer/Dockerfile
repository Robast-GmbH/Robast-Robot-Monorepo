FROM ros:humble-ros-core-jammy as base


 
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
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
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0


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
  wget \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

RUN pip3 install connexion[swagger-ui]

RUN apt-get update && apt-get install -y \
  # Install ros distro testing packages
  python3-autopep8 \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-diagnostic-updater
  
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

RUN apt-get update && apt-get upgrade -y

COPY "/entrypoint.sh" "/entrypoint.sh"
#ENTRYPOINT ["/entrypoint.sh"]



#watch -d -n 0.5 nvidia-smi

