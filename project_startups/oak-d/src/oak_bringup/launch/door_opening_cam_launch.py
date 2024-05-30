import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

sys.path.append(os.path.join(get_package_share_directory('oak_bringup'), 'launch'))
from utils.oak_launch_helper import generate_stereo_launch_description

def generate_launch_description():
    urdf_launch_dir = os.path.join(os.path.join(get_package_share_directory('oak_bringup'), 'launch'), 'door_opening_cam_urdf_launch.py')

    launch_configurations_oak_d = {
        'camera_model': 'OAK-D',
        'tf_prefix': 'oak',
        'base_frame': 'back_top_oak_d_camera',
        'parent_frame': 'door_opening_mechanism_link_y_axis_slide',
        'cam_pos_x': '-0.05',
        'cam_pos_y': '0.0',
        'cam_pos_z': '0.13721',
        'cam_roll': '0',
        'cam_pitch': '0.5235988',
        'cam_yaw': '3.14159',
        'mode': 'depth',
        'lrcheck': True,
        'extended': True,
        'subpixel': False,
        'confidence': 120,
        'LRchecktresh': 5,
        'monoResolution': '400p'
    }

    TF_TOPIC = "/tf"
    TF_STATIC_TOPIC = "/tf_static"
    NAMESPACE_ARM = "arm"
    remappings_tf = [
        (TF_TOPIC, "/" + NAMESPACE_ARM + TF_TOPIC),
        (TF_STATIC_TOPIC, "/" + NAMESPACE_ARM + TF_STATIC_TOPIC),
    ]

    actions_to_launch = generate_stereo_launch_description(launch_configurations_oak_d, urdf_launch_dir, NAMESPACE_ARM, remappings_tf)

    return LaunchDescription(actions_to_launch)
