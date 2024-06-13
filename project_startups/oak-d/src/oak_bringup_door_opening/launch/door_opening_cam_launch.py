import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

sys.path.append(os.path.join(get_package_share_directory('oak_bringup'), 'launch'))
from utils.oak_launch_helper import generate_stereo_launch_description

def generate_launch_description():
    urdf_launch_file = os.path.join(os.path.join(get_package_share_directory('oak_bringup_door_opening'), 'launch'), 'door_opening_cam_urdf_launch.py')

    oak_d_config_file = os.path.join(get_package_share_directory('oak_bringup_door_opening'), 'config', 'oak_d_config.yaml')

    TF_TOPIC = "/tf"
    TF_STATIC_TOPIC = "/tf_static"
    NAMESPACE_ARM = "arm"
    remappings_tf = [
        (TF_TOPIC, "/" + NAMESPACE_ARM + TF_TOPIC),
        (TF_STATIC_TOPIC, "/" + NAMESPACE_ARM + TF_STATIC_TOPIC),
    ]

    actions_to_launch = generate_stereo_launch_description(oak_d_config_file, urdf_launch_file, NAMESPACE_ARM, remappings_tf)

    return LaunchDescription(actions_to_launch)
