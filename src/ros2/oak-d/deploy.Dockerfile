FROM ghcr.io/robast-gmbh/monorepo/navigation_dev as build
COPY "." "/workspace/src/oak-d" 

WORKDIR /workspace
RUN rosdep update;\
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
RUN cd /workspace; \
    source /opt/ros/humble/setup.bash; \
    colcon build --cmake-args --packages-skip aws_hospital_world robast_map_update_module

FROM ros:humble-ros-core as final 
COPY --from=build /workspace/install /workspace/install 
COPY --from=build /workspace/src/oak-d/deploy-entrypoint.sh . 
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-requests 
    
ENTRYPOINT ["/deploy-entrypoint.sh" ]
