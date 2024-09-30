bash ping_and_wait.sh
docker pull ghcr.io/robast-gmbh/monorepo/robot_frontend:release
CURRENT_DIR=$(dirname "$(realpath "$0")")
docker run -it --name frontend_copier -v $CURRENT_DIR/:/app ghcr.io/robast-gmbh/monorepo/robot_frontend:release sh -c "cp -r /robot_frontend /app" 
docker container rm frontend_copier

robot_frontend/robot_frontend