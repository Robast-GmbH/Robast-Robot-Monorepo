FROM ghcr.io/robast-gmbh/monorepo/simulation as base


#install RMF
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y\
    ros-${ROS_DISTRO}-navigation2

RUN pip3 install uvicorn \
    fastapi\
    requests

RUN python3 -m pip install -U  cyclonedds nudged

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

