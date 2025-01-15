ARG ROS_BUILD_DISTRO=humble
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-base:${ROS_BUILD_DISTRO}
ARG ROS_BUILD_DISTRO

ENV PATH=/opt/ros/${ROS_BUILD_DISTRO}/bin:$PATH
ENV ROS_DOMAIN_ID=23
ENV ROS_DISTRO=${ROS_BUILD_DISTRO}
USER root


RUN apt-get update && apt-get install -y \
  pylint \
  python3-argcomplete \
  python3-autopep8 \
  python3-pybind11 \
  python3-vcstool \
  python3-venv \
  ros-${ROS_BUILD_DISTRO}-diagnostic-updater \
  ros-${ROS_BUILD_DISTRO}-rosbridge-server \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

WORKDIR /release
COPY . /release/src/robot_backend

RUN pip install -r src/robot_backend/requirements.txt && \
  rosdep update && \
  rosdep install --from-paths /release/src --ignore-src -r -y && \
  . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
  . /robast/${ROS_BUILD_DISTRO}/setup.sh && \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  bash /robast/humble/error_utils/bin/install_py_module.sh

WORKDIR /workspace
USER robast

RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USER}/.bashrc \
  && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/${USER}/.bashrc \
  && echo "if [ -f /release/install/setup.bash ]; then source /release/install/setup.bash; fi" >> /home/${USER}/.bashrc 
