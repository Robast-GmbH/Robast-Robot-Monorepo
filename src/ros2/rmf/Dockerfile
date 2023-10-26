FROM ros:humble-ros-base-jammy as devel

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
ARG DEBIAN_FRONTEND=

RUN echo "source install/setup.bash" >> /home/$USER/.bashrc


RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-ros-testing \
  ros-${ROS_DISTRO}-ament-cmake-test \
  ros-${ROS_DISTRO}-launch-testing \
  git \
  && rosdep init || echo "rosdep already initialized"

RUN pip3 install uvicorn \
    fastapi\
    requests

WORKDIR /robotnik
RUN git clone -b humble-devel https://github.com/RobotnikAutomation/robotnik_msgs.git && \
. /opt/ros/${ROS_DISTRO}/setup.sh && \
colcon build --packages-select robotnik_msgs --install-base /robast/${ROS_DISTRO}/ --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /workspace

RUN apt-get update \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USER/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /robotnik/install/setup.bash ]; then source /robotnik/install/setup.bash; fi" >> /home/$USER/.bashrc 



ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml

################Build######################################################
FROM ros:humble-ros-base-jammy as build

ARG GIT_TOKEN
ARG GIT_BRANCH=main
COPY --from=devel / /
COPY . /ws/src/rmf
RUN apt-get update && apt-get install -y git

WORKDIR /tmp
RUN git clone -b $GIT_BRANCH https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git

WORKDIR /ws
RUN mkdir -p /ws/src/ros2/; mkdir -p /ws/src/ros2/rmf/free_fleet/free_fleet; mkdir /robast; \
  mv /tmp/Monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y\
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  . /robast/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/

WORKDIR /robotnik
RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/

RUN  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/

############################## Release ##################################
FROM ros:humble-ros-base-jammy as release

COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast
COPY --from=build /robotnik /robotnik

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/robast/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

RUN apt-get update \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc \
  && echo "if [ -f /robotnik/install/setup.bash ]; then source /robotnik/install/setup.bash; fi" >> /home/$USER/.bashrc 