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

# Must compile moveit_task_constructor_msgs from source, because the ros package is not released yet (22.09.23)
# https://index.ros.org/p/moveit_task_constructor_msgs/
# TODO: Remove this once moveit_task_constructor_msgs is released as ros package 
RUN mkdir -p workspace_mtc/src && cd workspace_mtc/src && git init moveit_task_constructor \
    && cd /workspace_mtc/src/moveit_task_constructor && git remote add -f origin https://github.com/ros-planning/moveit_task_constructor.git \
    && git config core.sparseCheckout true \
    && echo "msgs" >> .git/info/sparse-checkout \
    && git pull origin humble \
    && apt update \
    && apt upgrade -y \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install --from-paths /workspace_mtc/src --ignore-src -r -y \
    && colcon build --build-base /workspace_mtc/build --install-base /workspace_mtc/install \
    && echo "if [ -f /workspace_mtc/install/setup.bash ]; then source /workspace_mtc/install/setup.bash; fi" >> /home/$USER/.bashrc