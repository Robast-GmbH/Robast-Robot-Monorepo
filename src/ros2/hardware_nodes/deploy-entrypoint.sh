#!/bin/bash

if [ -z ${DOCKER_MODE+x} ]; then
    cd /workspace 
    source install/setup.bash; \
    ros2 launch drawer_gate drawer_gate_launch.py
    tail -f /dev/null
 fi