import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('fleet_server'),
        'config',
        'params.yaml'
        )

    fleet_server_node = Node(
        package='fleet_server',
        executable='web_api',
        name='ff_server_api',
        parameters=[config]
    )

    return LaunchDescription([
     fleet_server_node
    ])
