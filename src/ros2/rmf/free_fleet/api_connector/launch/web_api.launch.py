from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():

    ff_server_api_node = Node(
        package='api_connector',
        executable='web_api',
        name='ff_server_api'
    )

    return LaunchDescription([
     ff_server_api_node
    ])
