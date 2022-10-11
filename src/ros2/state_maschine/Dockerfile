FROM ros:galactic-ros-core

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
  && rosdep init || echo "rosdep already initialized"

ARG GIT_BRANCH
ARG LOCAL_FOLDER_SOURCE=0

WORKDIR "/home/ros2_ws/src"

RUN echo "x23"
ADD . /home/ros2_ws/src/SMACC2
RUN if [ $LOCAL_FOLDER_SOURCE -eq 0 ] ; then (echo "downloading repo" && rm -R /home/ros2_ws/src/SMACC2  && git clone -b ${ROS_DISTRO} https://github.com/robosoft-ai/SMACC2.git /home/ros2_ws/src/SMACC2); else  (echo "using local copy" &&  rm -R /home/ros2_ws/src/SMACC2/.git); fi
WORKDIR "/home/ros2_ws"

# install dependencies and build
RUN ls src && vcs import  src --skip-existing --input src/SMACC2/SMACC2-not-released.${ROS_DISTRO}.repos \
  && ls src

RUN apt-get update && apt-get upgrade -y
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get update && apt-get install -q -y --no-install-recommends xterm

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --merge-install"

RUN curl -s https://1bde9b1a2e969457e8926f37a0a94a7f22c43a4056da8037:@packagecloud.io/install/repositories/robosoft-ai/SMACC2_RTA-startup/script.deb.sh | sudo bash

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt -y install ros-galactic-smacc2-rta

ENV USER robast
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc \
    echo "if [ -f /home/ros2_ws/install/setup.bash ]; then source /home/ros2_ws/install/setup.bash; fi" >> /home/$USER/.bashrc \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers