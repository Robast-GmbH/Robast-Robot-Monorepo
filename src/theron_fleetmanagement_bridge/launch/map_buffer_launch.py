from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='theron_fleetmanagement_bridge',
            executable='map_buffer',
            name='map_buffer'
        ),   
    ])