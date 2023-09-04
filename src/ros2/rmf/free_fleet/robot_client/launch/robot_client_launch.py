from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_client',
            namespace='rmf_robot_client',
            executable='robot_client',
            name='robot_client'
        ),
        
    ])