from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    start_nav_action_node = Node(
        package="nav_action_node",
        executable="nav_action_node",
    )

    ld = LaunchDescription()

    ld.add_action(start_nav_action_node)

    return ld
