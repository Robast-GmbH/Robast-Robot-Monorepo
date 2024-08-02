from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="disinfection_module",
                executable="disinfection_publisher",
                name="disinfection_publisher",
            ),
        ]
    )
