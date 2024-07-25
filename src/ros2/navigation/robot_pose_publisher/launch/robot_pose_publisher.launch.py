from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_robot_source_frame = LaunchConfiguration("use_robot_source_frame", default=True)

    start_robot_pose_publisher = Node(
        package="robot_pose_publisher",
        executable="robot_pose_publisher",
        parameters=[{"use_robot_source_frame": use_robot_source_frame}],
    )

    ld = LaunchDescription()

    ld.add_action(start_robot_pose_publisher)

    return ld
