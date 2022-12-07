#!/bin/bash
cd /workspace; 
source install/setup.bash; 

if [ -z ${DOCKER_MODE+x} ]; then
    ros2 run simple_fleetmanagement fleetmanagement
    tail -f /dev/null
 fi