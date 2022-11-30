#!/bin/bash
 cd  /workspace/src
 npm install -g serve
 npm ci
 npm run build&
 serve -s build&
if [ -z ${DOCKER_MODE+x} ]; then
    tail -f /dev/null
fi