FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  && rosdep init || echo "rosdep already initialized"

RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-smacc2 \
  ros-${ROS_DISTRO}-smacc2-dbgsym \
  ros-${ROS_DISTRO}-smacc2-msgs \
  ros-${ROS_DISTRO}-smacc2-msgs-dbgsym

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp