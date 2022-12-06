#!/bin/bash
cd /workspace; \
source install/setup.bash; \
ros2 run simple_fleetmanagement fleetmanagement
if [ -z ${DOCKER_MODE+x} ]; then
    tail -f /dev/null
 fi