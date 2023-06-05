FROM ghcr.io/robast-gmbh/monorepo/simulation:latest

# Adding moveit2_tutorials as separate workspace which can be used for learning and testing purposed.
# Can be removed once there is enough moveit2 skill and knowledge archieved
RUN mkdir -p workspace_moveit2/src && cd workspace_moveit2/src && git clone https://github.com/ros-planning/moveit2_tutorials -b humble --depth 1 \
  && vcs import < moveit2_tutorials/moveit2_tutorials.repos \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && . /workspace_ros_gz/install/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_moveit2/src --ignore-src -r -y \
  && colcon build --parallel-workers 3 --build-base /workspace_moveit2/build --install-base /workspace_moveit2/install \
  && echo "if [ -f /workspace_moveit2/install/setup.bash ]; then source /workspace_moveit2/install/setup.bash; fi" >> /home/$USER/.bashrc
