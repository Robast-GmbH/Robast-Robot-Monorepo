from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    nav_action_node = Node(
        package="nav_action_node",
        executable="nav_action_node",
    )

    dispenser_ingestor_mock = Node(
        package="dispenser_ingestor_mock",
        executable="dispenser_ingestor_mock",
    )

    robot_pose_publisher = Node(
        package="robot_pose_publisher",
        executable="robot_pose_publisher",
    )

    return LaunchDescription(
        [
            nav_action_node,
            dispenser_ingestor_mock,
            robot_pose_publisher,
        ]
    )
