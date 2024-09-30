bash ping_and_wait.sh
docker pull ghcr.io/robast-gmbh/monorepo/robot_frontend:release

CONTAINER_NAME="frontend_copier"
CURRENT_DIR=$(dirname "$(realpath "$0")")
TARGET_DIR="/app"
docker run -it --name $CONTAINER_NAME -v $CURRENT_DIR/:$TARGET_DIR ghcr.io/robast-gmbh/monorepo/robot_frontend:release sh -c "cp -r /robot_frontend $TARGET_DIR" 
docker container rm $CONTAINER_NAME

robot_frontend/robot_frontend