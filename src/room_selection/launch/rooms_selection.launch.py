from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='room_selection',
            executable='room_selection_nav_goal',
            name='room_selection_nav_goal',
            output='screen'),
    ])