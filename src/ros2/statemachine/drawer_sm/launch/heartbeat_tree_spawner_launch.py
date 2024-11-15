from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bt_node = Node(
        package="drawer_sm",
        executable="heartbeat_tree_spawner",
        name="heartbeat_tree_spawner_node",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(bt_node)

    return ld
