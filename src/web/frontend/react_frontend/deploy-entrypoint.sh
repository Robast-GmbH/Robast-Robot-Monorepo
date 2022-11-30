#!/bin/bash
 cd  /workspace/src
 npm install -g serve
 npm ci
 npm run build&
 serve -s build&
if $DOCKER_RUNMODE == "Infinite"
then
  tail -f /dev/null
fi