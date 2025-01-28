from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    initial_pose_publisher = Node(
        package="initial_pose_publisher",
        executable="initial_pose_publisher",
    )

    ld = LaunchDescription()

    ld.add_action(initial_pose_publisher)

    return ld
