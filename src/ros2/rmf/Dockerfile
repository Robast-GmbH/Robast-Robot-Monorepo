FROM ghcr.io/robast-gmbh/monorepo/simulation as base


#install RMF
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN pip3 install uvicorn \
    fastapi

RUN python3 -m pip install -U  cyclonedds nudged

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

