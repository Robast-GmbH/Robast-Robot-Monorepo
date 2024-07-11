from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drawer_mock = Node(
        package="drawer_mock",
        executable="drawer_mock",
    )

    ld = LaunchDescription()
    ld.add_action(drawer_mock)
    return ld
