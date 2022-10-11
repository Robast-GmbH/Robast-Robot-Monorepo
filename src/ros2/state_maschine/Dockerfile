FROM ros:galactic-ros-core

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  dirmngr \
  git \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  && rosdep init || echo "rosdep already initialized"

# RUN apt-get update && apt-get install -y \
#   ros-${ROS_DISTRO}-smacc2 \
#   ros-${ROS_DISTRO}-smacc2-dbgsym \
#   ros-${ROS_DISTRO}-smacc2-msgs \
#   ros-${ROS_DISTRO}-smacc2-msgs-dbgsym

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




#RUN mkdir SMACC2 && cd SMACC2 && git clone https://github.com/robosoft-ai/SMACC2.git \
 # && apt update \
 # && apt upgrade -y \
  # && . /opt/ros/${ROS_DISTRO}/setup.bash \
  #&& rosdep update \
 # && vcs import /SMACC2 --skip-existing --input /SMACC2/SMACC2/SMACC2.galactic.repos \
  #&& rosdep install --from-paths /SMACC2 --ignore-src -r -y 
  #  colcon build --build-base /workspace/build --install-base /workspace/install \
  

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
