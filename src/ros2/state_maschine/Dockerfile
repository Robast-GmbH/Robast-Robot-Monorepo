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

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --merge-install --parallel-workers 16 --packages-skip sm_advanced_recovery_1 sm_atomic sm_atomic_24hr sm_atomic_hierarchy sm_autoware_avp sm_aws_warehouse_navigation sm_branching sm_dance_bot sm_dance_bot_strikes_back sm_dance_bot_warehouse sm_dance_bot_warehouse_2 sm_dance_bot_warehouse_3 sm_drawer_gate sm_ferrari sm_husky_barrel_search_1 sm_multi_stage_1 sm_multi_ur5_sim sm_pack_ml sm_pubsub_1 sm_respira_1 sm_test_moveit_ur5_sim sm_three_some"

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