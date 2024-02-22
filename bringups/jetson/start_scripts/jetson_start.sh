#!/bin/sh

source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i \
    publish_map_tf:=false \
    publish_tf:=false \
    xacro_path:=/zed_setup/zed_descr.urdf.xacro \
    config_path:=/zed_setup/common.yaml 

