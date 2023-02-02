FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces  AS communication_interfaces

FROM ghcr.io/robast-gmbh/monorepo/simple_fleetmanagement_dev  AS build

COPY "." "/workspace/src/fleetmanagement" 
COPY --from=communication_interfaces /communication_interfaces workspace/src/communication_interfaces

WORKDIR /workspace
RUN rosdep update;\
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source /opt/ros/humble/setup.bash; \
    colcon build --packages-select communication_interfaces; \
    colcon build --packages-select simple_fleetmanagement;


FROM ros:humble-ros-core AS final
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-rosdep \
    python3-requests \
    && rosdep init || echo "rosdep already initialized"
    
COPY --from=build /workspace/install /workspace/install 
COPY --from=build /workspace/src/fleetmanagement/deploy-entrypoint.sh / 
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/deploy-entrypoint.sh" ]