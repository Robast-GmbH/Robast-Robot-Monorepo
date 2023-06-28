#!/bin/bash
cd /workspace; 
source install/setup.bash; 

if [ -z ${DOCKER_MODE+x} ]; then
    tail -f /dev/null
 fi