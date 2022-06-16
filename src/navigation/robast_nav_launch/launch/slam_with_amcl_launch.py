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

    # slam
    slam_base_launch_file = LaunchConfiguration('slam_base_launch_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_posegraph = LaunchConfiguration('slam_posegraph')
    slam_executable = LaunchConfiguration('slam_executable')

    # amcl
    amcl_base_launch_file = LaunchConfiguration('amcl_base_launch_file')

    declare_slam_toolbox_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(robast_nav_launch_dir, 'config', 'slam_toolbox_params_offline.yaml'),
        description='Full path to the ROS 2 slam params file to use')

    declare_slam_base_launch_file_cmd = DeclareLaunchArgument(
        'slam_base_launch_file',
        default_value=os.path.join(robast_nav_launch_dir, 'launch', 'slam_toolbox_base_launch.py'),
        description='Full path to the slam_toolbox_base_launch')

    declare_slam_posegraph_file_cmd = DeclareLaunchArgument(
        'slam_posegraph',
        default_value=os.path.join(robast_nav_launch_dir, 'maps', '5OG'),
        description='Full path to the slam_toolbox posegraph map file to use')

    declare_slam_executable_cmd = DeclareLaunchArgument(
        'slam_executable',
        default_value='sync_slam_toolbox_node',
        description='kind of slam toolbox node'
    )

    # amcl
    declare_amcl_toolbox_params_file_cmd = DeclareLaunchArgument(
        'amcl_base_launch_file',
        default_value=os.path.join(robast_nav_launch_dir, 'launch', 'amcl_launch.py'),
        description='Full path to the AMCL launch file')

    # set arguemnts for the slam launch file
    slam_arguments = {
        'slam_executable': slam_executable,
        'slam_params_file': slam_params_file,
        'slam_posegraph': slam_posegraph,
        'slam_mode': 'mapping',
        'slam_map_topic': '/slam_map',
        'transform_publish_period': '0.00',
        # 'namespace': 'slam'
    }.items()

    amcl_arguments = {
        'map_server_map_topic': 'base_map'
        # 'namespace': 'amcl'
    }.items()

    start_parallel_mapping = Node(
        package='robast_map_update_module',
        executable='map_combine_node',
        name='map_combine_node',
        output='screen',
    )

    slam_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_base_launch_file), launch_arguments=slam_arguments)

    amcl_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_base_launch_file),  launch_arguments=amcl_arguments)

    ld = LaunchDescription()

    # declare parameters
    ld.add_action(declare_slam_toolbox_params_file_cmd)
    ld.add_action(declare_slam_posegraph_file_cmd)
    ld.add_action(declare_slam_executable_cmd)
    ld.add_action(declare_slam_base_launch_file_cmd)
    ld.add_action(declare_amcl_toolbox_params_file_cmd)

    # import other launch files
    ld.add_action(start_parallel_mapping)
    ld.add_action(amcl_base_launch)
    ld.add_action(slam_base_launch)

    return ld
