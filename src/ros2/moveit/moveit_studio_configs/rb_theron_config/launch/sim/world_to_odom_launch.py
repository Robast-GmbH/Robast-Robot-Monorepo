from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
    )

    nodes_to_launch = [
        static_tf_node,
    ]

    return LaunchDescription(nodes_to_launch)
