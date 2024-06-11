from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    start_image_transformer = Node(
        package='image_transformer',
        executable='image_transformer_node'
    )

    ld = LaunchDescription()

    ld.add_action(start_image_transformer)

    return ld

