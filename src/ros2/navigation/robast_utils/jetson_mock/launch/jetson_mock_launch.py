import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetson_mock',
            executable='zed_mock_node',
            name='zed_mock_node',
            output='screen'
        )
    ])