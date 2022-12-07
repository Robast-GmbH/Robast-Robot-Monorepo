#!/bin/bash
 cd  /workspace/src
 npm install -g serve
 npm ci 
 npm run build&
 
 if [ -z ${DOCKER_MODE+x} ]; then
   serve -s build&
   tail -f /dev/null
 fi