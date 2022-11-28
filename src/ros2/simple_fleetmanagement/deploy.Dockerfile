FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces_deploy  AS communication_interfaces


FROM ros:humble-ros-core AS build
RUN apt-get update && apt-get install -y \
 build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  cmake \
  gdb \
  git \
  pylint \ 
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-requests \
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
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "cd ~/../../workspaces/Robast_RosTheron" >> /home/$USER/.bashrc \
  && echo 'export PS1="\[\e[31m\][\[\e[m\]\[\e[38;5;172m\]\u\[\e[m\]@\[\e[38;5;153m\]\h\[\e[m\]\[\e[38;5;214m\]\W\[\e[m\]\[\e[31m\]]\[\e[m\]\\$"' >> /home/$USER/.bashrc

RUN apt-get update && apt-get upgrade -y


COPY "." "/workspace/src/fleetmanagement" 
COPY --from=communication_interfaces /communication_interfaces workspace/src/communication_interfaces

WORKDIR /workspace
RUN rosdep update;\
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source /opt/ros/humble/setup.bash; \
    colcon build --packages-select communication_interfaces; \
    colcon build --packages-select simple_fleetmanagement;


FROM ros:humble-ros-core AS final
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-rosdep \
    python3-requests \
    && rosdep init || echo "rosdep already initialized"
COPY --from=build /workspace/install /workspace/install 

SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source install/setup.bash; \
    ros2 run simple_fleetmanagement fleetmanagement

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CMD tail -f /dev/null