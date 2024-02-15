#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 bag record -o /workspace/bags/robot_imu_2.db3 \
    tf \
    tf_static \
    robot/robotnik_base_control/odom \
    robot/imu/data \
    robot/vectornav/imu/data\
    robot/pad_teleop/cmd_vel \
    robot/vectornav/odom \
    clock