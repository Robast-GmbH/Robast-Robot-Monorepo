from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():

    fleet_server_node = Node(
        package='fleet_server',
        executable='web_api',
        name='ff_server_api'
    )

    return LaunchDescription([
     fleet_server_node
    ])
