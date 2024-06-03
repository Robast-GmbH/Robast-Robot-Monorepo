import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

sys.path.append(os.path.join(get_package_share_directory('oak_bringup'), 'launch'))
from utils.oak_launch_helper import generate_stereo_launch_description


def generate_launch_description():
    urdf_launch_file = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch', 'urdf_launch.py')

    oak_d_config_file = os.path.join(get_package_share_directory('oak_bringup'), 'config', 'oak_d_config.yaml')

    namespace = ''
    remappings = []

    actions_to_launch = generate_stereo_launch_description(oak_d_config_file, urdf_launch_file, namespace, remappings)

    return LaunchDescription(actions_to_launch)
