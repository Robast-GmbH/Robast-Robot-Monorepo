import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    feuerplan_publisher_dir = get_package_share_directory('feuerplan_publisher')

    default_feuerplan_image_path = os.path.join(feuerplan_publisher_dir, 'resources','slide1.jpg')

    feuerplan_path = LaunchConfiguration('feuerplan_path',  default = default_feuerplan_image_path)
    
    declare_feuerplan_image_cmd = DeclareLaunchArgument(
        'feuerplan_path',
        default_value=feuerplan_path,
        description='Path to the feuerplan image')
    
    start_feuerplan_publisher = Node(
        package='feuerplan_publisher',
        executable='feuerplan_publisher_node',
        parameters=[
                    {'feuerplan_path': feuerplan_path}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_feuerplan_image_cmd)

    ld.add_action(start_feuerplan_publisher)

    return ld