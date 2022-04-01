FROM ros:foxy-ros-core-focal

RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
  
# install some pip packages needed for testing
RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest

# install Fast-RTPS dependencies
RUN sudo apt-get install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev

# install Cyclone DDS dependencies
  RUN sudo apt-get install --no-install-recommends -y \
      libcunit1-dev