import os
from typing import Mapping

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robast_nav_launch_dir = get_package_share_directory('robast_nav_launch')
    world_posegraph = LaunchConfiguration('world_model')

    declare_world_model_cmd = DeclareLaunchArgument(
        'world_posegraph',
        default_value=os.path.join(robast_nav_launch_dir, 'maps', '5OG'),
        description='math to the world model'
    )
    slam_toolbox_params_yaml = os.path.join(robast_nav_launch_dir, 'config', 'slam_toolbox_params_offline.yaml')
    slam_launch_file = os.path.join(robast_nav_launch_dir, 'launch', 'slam_toolbox_base_launch.py')

    slam_arguments = {
        'slam_executable': 'sync_slam_toolbox_node',
        'slam_params_file': slam_toolbox_params_yaml,
        'slam_posegraph': world_posegraph,
        'slam_mode': 'mapping',
        'slam_map_topic': '/map',
    }.items()

    launch_slam_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file), launch_arguments=slam_arguments)

    ld = LaunchDescription()
    ld.add_action(declare_world_model_cmd)
    ld.add_action(launch_slam_base_launch)

    return ld
