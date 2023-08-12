FROM ghcr.io/robast-gmbh/monorepo/simulation as base


#install RMF
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y\
    ros-${ROS_DISTRO}-rmf-fleet-msgs\
    ros-${ROS_DISTRO}-navigation2

RUN pip3 install uvicorn \
    fastapi\
    requests

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


WORKDIR /ws
RUN mkdir -p /ws/src/ros2/; mkdir -p /ws/src/ros2/rmf/free_fleet/free_fleet; mkdir /robast; \
  mv /tmp/Monorepo/src/ros2/communication_interfaces /ws/src/ros2/communication_interfaces \
  && mv /tmp/Monorepo/src/ros2/dds_configs /robast/dds_configs
  

RUN rosdep update && \
  rosdep install --from-paths /ws/src --ignore-src -r -y

WORKDIR /tmp
RUN git clone https://github.com/open-rmf/free_fleet.git

WORKDIR /ws
RUN mv /tmp/free_fleet /ws/src/ros2/rmf/free_fleet/free_fleet

RUN  . /opt/ros/${ROS_DISTRO}/setup.sh && \
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