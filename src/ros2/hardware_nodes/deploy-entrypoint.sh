#!/bin/bash

if [ -z ${DOCKER_MODE+x} ]; then
    tail -f /dev/null
 fi