CURRENT_DIR=$(dirname "$(realpath "$0")")
docker run -it --name frontend_copier -v $CURRENT_DIR/:/app frontend_copy sh -c "cp -r /robot_frontend /app" 
docker container rm frontend_copier

robot_frontend/robot_frontend
