FROM ros:foxy-ros-core-focal

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  # nav2-util is needed for the SimpleActionServer
  ros-${ROS_DISTRO}-nav2-util
