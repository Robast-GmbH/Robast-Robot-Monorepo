#!/bin/sh

. install/setup.sh
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i \
    camera_name:=robot/zed \
    publish_map_tf:=false \
    publish_tf:=false \
    publish_imu_tf:=false \
    sim_mode:=false \
    xacro_path:=/zed_setup/zed_descr.urdf.xacro \
    config_path:=/zed_setup/common.yaml \
    publish_urdf:=false

