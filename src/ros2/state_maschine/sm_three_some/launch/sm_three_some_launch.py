from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    sm_three_some_cmd = Node(
        package='sm_three_some',
        executable='sm_three_some_node',
        name='sm_three_some',
        output='screen'
    )

    keyboard_server_cmd = Node(
        package='keyboard_client',
        executable='keyboard_server_node.py',
        name='keyboard_server_node',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(sm_three_some_cmd)
    ld.add_action(keyboard_server_cmd)

    return ld