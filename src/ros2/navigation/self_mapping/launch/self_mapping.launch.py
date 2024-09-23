from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='self_mapping',
            executable='self_mapping_node',
            name='self_mapping',
            output='screen'
        )
    ])