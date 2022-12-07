#!/bin/bash

if [ -z ${DOCKER_MODE+x} ]; then
    python /workspace/src/main.py&
    tail -f /dev/null
 fi