FROM ghcr.io/robast-gmbh/monorepo/simulation:latest

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