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

