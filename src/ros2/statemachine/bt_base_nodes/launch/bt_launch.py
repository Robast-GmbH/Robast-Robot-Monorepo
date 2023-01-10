from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drawer_sm',
            executable='drawer_sm2',
            name='drawer_sm2',
        )
    ])