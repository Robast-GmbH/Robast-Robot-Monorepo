import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    feuerplan_publisher_dir = get_package_share_directory('feuerplan_publisher')

    default_resources_path = os.path.join(feuerplan_publisher_dir, 'resources')

    feuerplan_image = LaunchConfiguration('feuerplan_image_path',  default = "slide1.jpg")
    resource_base_folder = LaunchConfiguration('resource_base_folder', default = default_resources_path)

    declare_resource_base_folder_cmd = DeclareLaunchArgument(
        'resource_base_folder',
        default_value=resource_base_folder,
        description='Path to the resources folder which contains the default blobs for the network')
    
    declare_feuerplan_image_cmd = DeclareLaunchArgument(
        'feuerplan_image',
        default_value=feuerplan_image,
        description='Path to the object detection blob needed for detection')
    
    start_feuerplan_publisher = Node(
        package='feuerplan_publisher',
        executable='feuerplan_publisher_node',
        parameters=[
                    {'feuerplan_image': feuerplan_image},
                    {'resource_base_folder': resource_base_folder},
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_resource_base_folder_cmd)
    ld.add_action(declare_feuerplan_image_cmd)

    ld.add_action(start_feuerplan_publisher)

    return ld

