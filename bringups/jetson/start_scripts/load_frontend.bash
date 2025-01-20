#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash $SCRIPT_DIR/ping_and_wait.sh
docker pull ghcr.io/robast-gmbh/monorepo/robot_frontend:release

CONTAINER_NAME="frontend_copier"
TARGET_DIR="/app"
docker run -it --name $CONTAINER_NAME -v $SCRIPT_DIR/:$TARGET_DIR ghcr.io/robast-gmbh/monorepo/robot_frontend:release sh -c "cp -r /robot_frontend $TARGET_DIR" 
docker container rm $CONTAINER_NAME
$SCRIPT_DIR/robot_frontend/robot_frontend
