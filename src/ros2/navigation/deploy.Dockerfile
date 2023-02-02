FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces_deploy  AS communication_interfaces

FROM ghcr.io/robast-gmbh/monorepo/navigation_dev as build
COPY "." "/workspace/src/navigation" 
COPY --from=communication_interfaces /communication_interfaces workspace/src/communication_interfaces

WORKDIR /workspace
RUN rosdep update;\
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source /opt/ros/humble/setup.bash; \
    colcon build --cmake-args --packages-skip aws_hospital_world robast_map_update_module
COPY ./environment_vars.yaml /workspace/environment_vars.yaml


FROM ros:humble-ros-core as final 
COPY --from=build /workspace/install /workspace/install 
COPY --from=build /workspace/src/navigation/environment_vars.yaml /workspace/environment_vars.yaml
COPY --from=build /workspace/src/navigation/deploy-entrypoint.sh . 
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2\ 
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-requests 
ENTRYPOINT ["/deploy-entrypoint.sh" ]
