import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    robot_localization_params_yaml = os.path.join(get_package_share_directory('map_server'), 'config', 'robot_localization_ekf.yaml')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_params_yaml]),
    ])
