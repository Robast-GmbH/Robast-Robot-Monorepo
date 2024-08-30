#!/bin/bash
START_TIME=$(date +%s%N)

source /opt/ros/humble/setup.bash
ros2 topic echo /robot/zed/zed_node/rgb_raw/camera_info --once > /dev/null 2>&1

if [ $? -eq 0 ]; then
    END_TIME=$(date +%s%N)
    
    DURATION=$(( (END_TIME - START_TIME) / 1000000 ))
    if [ $DURATION -gt 2000 ]; then
        printf "Timeout: %d ms\n" $DURATION
        exit 1  # Timeout: unhealthy
    else
        exit 0  
    fi
else
    exit 1 
fi