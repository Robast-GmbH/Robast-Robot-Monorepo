import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

sys.path.append(os.path.join(get_package_share_directory('oak_bringup'), 'launch'))
from utils.oak_launch_helper import generate_stereo_launch_description


def generate_launch_description():
    urdf_launch_file = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch', 'urdf_launch.py')

    launch_configurations_oak_d = {
        'camera_model': 'OAK-D',
        'tf_prefix': 'oak_s',
        'base_frame': 'oak-d_frame',
        'parent_frame': 'robot/base_link',
        'cam_pos_x': '0.317',
        'cam_pos_y': '-0.197',
        'cam_pos_z': '1.3044',
        'cam_roll': '0.0',
        'cam_pitch': '0.0',
        'cam_yaw': '0.0',
        'mode': 'depth',
        'lrcheck': True,
        'extended': False,
        'subpixel': True,
        'confidence': 120,
        'LRchecktresh': 5,
        'monoResolution': '720p'
    }

    namespace = ''
    remappings = []

    actions_to_launch = generate_stereo_launch_description(launch_configurations_oak_d, urdf_launch_file, namespace, remappings)

    return LaunchDescription(actions_to_launch)
