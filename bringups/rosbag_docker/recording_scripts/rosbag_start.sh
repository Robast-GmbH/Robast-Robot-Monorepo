#!/bin/bash

source /opt/ros/humble/setup.bash
ros2 bag record --max-bag-size 1000000000 -o /workspace/bags/hagen1.db3 \
    tf \
    tf_static \
    /robot/bpearl_laser/scan \
    /robot/front_laser/scan \
    /robot/robotnik_base_control/odom
    # /map \
    # /cmd_vel \
    # /stereo/door_handle_position \
    # stereo/depth \
    # color/image




