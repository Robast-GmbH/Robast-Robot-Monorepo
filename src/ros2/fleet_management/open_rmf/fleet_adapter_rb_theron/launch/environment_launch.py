from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ros_bridge_share_dir = get_package_share_directory("rosbridge_server")

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            f"{ros_bridge_share_dir}/launch/rosbridge_websocket_launch.xml"
        )
    )

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
            rosbridge_launch,
            nav_action_node,
            dispenser_ingestor_mock,
            robot_pose_publisher,
        ]
    )
