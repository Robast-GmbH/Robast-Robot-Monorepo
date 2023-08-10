FROM ghcr.io/robast-gmbh/monorepo/simulation as base


#install RMF
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y\
    ros-${ROS_DISTRO}-rmf-api-msgs\
    ros-${ROS_DISTRO}-rmf-utils\
    ros-${ROS_DISTRO}-rmf-robot-sim-common\
    ros-${ROS_DISTRO}-rmf-robot-sim-gz-plugins\
    ros-${ROS_DISTRO}-rmf-scheduler-msgs \
    ros-${ROS_DISTRO}-rmf-battery \
    ros-${ROS_DISTRO}-rmf-building-map-msgs\
    ros-${ROS_DISTRO}-rmf-building-map-tools\
    ros-${ROS_DISTRO}-rmf-building-sim-gz-plugins\
    ros-${ROS_DISTRO}-rmf-task\
    ros-${ROS_DISTRO}-rmf-task-msgs\
    ros-${ROS_DISTRO}-rmf-task-ros2\
    ros-${ROS_DISTRO}-rmf-task-sequence\
    ros-${ROS_DISTRO}-rmf-fleet-adapter\
    ros-${ROS_DISTRO}-rmf-fleet-adapter-python\
    ros-${ROS_DISTRO}-rmf-fleet-msgs\
    ros-${ROS_DISTRO}-rmf-traffic-ros2\
    ros-${ROS_DISTRO}-rmf-traffic\
    ros-${ROS_DISTRO}-rmf-traffic-editor\
    ros-${ROS_DISTRO}-rmf-traffic-msgs\ 
    ros-${ROS_DISTRO}-rmf-visualization\
    ros-${ROS_DISTRO}-rmf-visualization-msgs\
    ros-${ROS_DISTRO}-rmf-visualization-fleet-states\
    ros-${ROS_DISTRO}-rmf-visualization-floorplans\
    ros-${ROS_DISTRO}-rmf-visualization-rviz2-plugins\
    ros-${ROS_DISTRO}-rmf-websocket\
    ros-${ROS_DISTRO}-navigation2

RUN pip3 install uvicorn \
    fastapi

RUN python3 -m pip install -U  cyclonedds nudged

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#######################################
############Tobi was here##############
###TODO remove everything below this###
#######################################

FROM ros:humble-ros-base-jammy as build

ARG GIT_TOKEN
ARG GIT_BRANCH=dev/hagen
COPY --from=base / /
COPY . /ws/src/rmf
RUN apt-get update && apt-get install -y git

WORKDIR /tmp
RUN git clone -b $GIT_BRANCH https://$GIT_TOKEN@github.com/Robast-GmbH/Monorepo.git 
RUN git clone https://github.com/open-rmf/free_fleet.git


WORKDIR /ws
RUN mkdir -p /ws/src/ros2/; mkdir /robast; mv /tmp/Monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs \
  && mv /tmp/Monorepo/src/ros2/simulation/tiplu_world /ws/src/ros2/simulation \
  && mv /tmp/Monorepo/src/ros2/robot_description /ws/src/ros2/robot_description \
  && mv /tmp/free_fleet /ws/ros2/rmf/free_fleet/free_fleet

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build --install-base /robast/${ROS_DISTRO}/

############################## Hagen ##################################
FROM ros:humble-ros-base-jammy as hagen

COPY --from=base / /
COPY --from=build /opt/ros /opt/ros
COPY --from=build /robast /robast

RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENV ROS_DOMAIN_ID=0
RUN apt-get update \
  && echo "if [ -f /robast/${ROS_DISTRO}/setup.bash ]; then source /robast/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc