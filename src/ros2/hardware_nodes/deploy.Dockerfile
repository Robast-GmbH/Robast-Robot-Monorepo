FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces AS communication_interfaces
FROM ghcr.io/robast-gmbh/monorepo/hardware_libs AS libs


FROM ghcr.io/robast-gmbh/monorepo/hardware_nodes_dev As build
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
COPY --from=build /workspace/src/hardware_nodes/deploy-entrypoint.sh .
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/deploy-entrypoint.sh" ]