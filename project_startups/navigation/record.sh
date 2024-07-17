#!/bin/bash

source /opt/ros/humble/setup.bash
ros2 bag record -o /workspace/bags/slam_map.db3 \
    tf \
    tf_static \
    robot/robotnik_base_control/odom \
    robot/imu/data \
    map\
    robot/front_laser/scan \
