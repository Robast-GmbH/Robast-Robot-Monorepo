FROM ghcr.io/robast-gmbh/monorepo/communication_interfaces_deploy  AS communication_interfaces

FROM ros:humble-ros-core AS build


RUN apt-get update && apt-get install -y \
  cmake \
  python3-argcomplete \
  python3-pip \
  python3-rosdep \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

RUN pip3 install connexion[swagger-ui]

RUN apt-get update && apt-get install -y \
    python3-autopep8 \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 
  
RUN apt-get update && apt-get upgrade -y
COPY "." "/workspace/src/fleetmanagement" 
COPY --from=communication_interfaces /communication_interfaces workspace/src/communication_interfaces
WORKDIR workspace/
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash\
    colcon build