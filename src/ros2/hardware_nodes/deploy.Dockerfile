FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces_deploy  AS communication_interfaces
FROM ghcr.io/robast-gmbh/monorepo/hardware_libs_deploy AS libs


FROM ros:humble-ros-core As build
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libboost-all-dev \
    ros-humble-ros-testing \
    ros-humble-ament-cmake-test \
    ros-humble-launch-testing \
    && rosdep init || echo "rosdep already initialized"

COPY "." "/workspace/src/hardware_nodes" 
COPY --from=communication_interfaces /communication_interfaces workspace/src/communication_interfaces
COPY --from=libs /libs workspace/libs

WORKDIR /workspace
RUN rosdep update;\
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source /opt/ros/humble/setup.bash; \
    colcon build --continue-on-error


FROM ros:humble-ros-core As final
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-rosdep \
    && rosdep init || echo "rosdep already initialized"
COPY --from=build /workspace/install /workspace/install

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/workspace/src/deploy-entrypoint.sh" ]