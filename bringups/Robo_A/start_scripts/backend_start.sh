#!/bin/bash

cd /release
source install/setup.bash

# Start the ros2 command in a new process group
setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Save the PGID of the new process group
rosbridge_pgid=$!

# Register a function to run when the script exits
trap "kill -9 -$rosbridge_pgid" EXIT

# Start the Python script
python3 src/robot_backend/main.py

# while sleep 1000; do :; done