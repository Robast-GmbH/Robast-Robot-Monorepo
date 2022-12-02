#!/bin/bash
 cd  /workspace/src
 npm install -g serve
 
 
 if [ -z ${DOCKER_MODE+x} ]; then
   npm ci 
   npm run build&
   serve -s build&
   tail -f /dev/null
 fi