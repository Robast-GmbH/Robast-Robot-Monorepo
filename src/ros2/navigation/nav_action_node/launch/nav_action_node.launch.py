import math

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    sector_angle = LaunchConfiguration('sector_angle',  default = math.pi / 4)
    sector_radius = LaunchConfiguration('sector_radius',  default = 0.8)
    free_space_threshold = LaunchConfiguration('free_space_threshold',  default = 80.0)

    declare_sector_angle_cmd = DeclareLaunchArgument(
        'sector_angle',
        default_value=sector_angle,
        description='Confidence threshold for matches')
    
    declare_sector_radius_cmd = DeclareLaunchArgument(
        'sector_radius',
        default_value=sector_radius,
        description='Confidence threshold for matches')
    
    declare_free_space_threshold_cmd = DeclareLaunchArgument(
        'free_space_threshold',
        default_value=free_space_threshold,
        description='Confidence threshold for matches')

    start_nav_action_node = Node(
        package="nav_action_node",
        executable="nav_action_node",
    )

    start_sector_check_node = Node(
        package="nav_action_node",
        executable="sector_check_node",
        parameters=[
                    {'sector_angle': sector_angle},
                    {'sector_radius': sector_radius},
                    {'free_space_threshold': free_space_threshold}
        ],
    )


    ld = LaunchDescription()

    ld.add_action(declare_sector_angle_cmd)
    ld.add_action(declare_sector_radius_cmd)
    ld.add_action(declare_free_space_threshold_cmd)
    ld.add_action(start_nav_action_node)
    ld.add_action(start_sector_check_node)

    return ld
