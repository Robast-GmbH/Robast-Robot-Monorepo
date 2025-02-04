from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    initial_pose_publisher = Node(
        package="qr_code_scanner",
        executable="qr_code_scanner",
    )

    ld = LaunchDescription()

    ld.add_action(initial_pose_publisher)

    return ld
