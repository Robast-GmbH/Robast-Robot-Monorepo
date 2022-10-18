from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    sm_drawer_gate_cmd = Node(
        package='sm_drawer_gate',
        executable='sm_drawer_gate_node',
        name='sm_drawer_gate',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(sm_drawer_gate_cmd)

    return ld