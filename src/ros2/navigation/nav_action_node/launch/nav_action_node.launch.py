import os
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    is_simulation = os.environ["is_simulation"]

    if is_simulation == 'True':
            robot_cmd_vel_topic = "diff_drive_base_controller/cmd_vel_unstamped"
            robot_base_frame = "base_link"
    else:
            robot_cmd_vel_topic = "robot/robotnik_base_control/cmd_vel"
            robot_base_frame = "robot/base_link"

    sector_angle = LaunchConfiguration('sector_angle',  default = math.pi / 4) # 45 degrees
    sector_radius = LaunchConfiguration('sector_radius',  default = 0.8)
    free_space_threshold = LaunchConfiguration('free_space_threshold',  default = 80.0)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic',  default = robot_cmd_vel_topic)
    robot_base_frame_param = LaunchConfiguration('robot_base_frame_param',  default = robot_base_frame)

    declare_sector_angle_cmd = DeclareLaunchArgument(
        'sector_angle',
        default_value=sector_angle,
        description='Sector angle for sector check')
    
    declare_sector_radius_cmd = DeclareLaunchArgument(
        'sector_radius',
        default_value=sector_radius,
        description='Sector radius for sector check')
    
    declare_free_space_threshold_cmd = DeclareLaunchArgument(
        'free_space_threshold',
        default_value=free_space_threshold,
        description='Free space threshold for sector check')
    
    declare_robot_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value=cmd_vel_topic,
        description='Robot cmd_vel topic')
    
    declare_robot_base_frame_cmd = DeclareLaunchArgument(
        'robot_base_frame',
        default_value=robot_base_frame_param,
        description='Robot base frame')

    start_nav_action_node = Node(
        package="nav_action_node",
        executable="nav_action_node",
        parameters=[
                    {'sector_angle': sector_angle},
                    {'sector_radius': sector_radius},
                    {'free_space_threshold': free_space_threshold},
                    {'cmd_vel_topic': cmd_vel_topic},
                    {'robot_base_frame_param': robot_base_frame_param},
        ],
    )


    ld = LaunchDescription()

    ld.add_action(declare_sector_angle_cmd)
    ld.add_action(declare_sector_radius_cmd)
    ld.add_action(declare_free_space_threshold_cmd)
    ld.add_action(declare_robot_cmd_vel_topic_cmd)
    ld.add_action(declare_robot_base_frame_cmd)
    ld.add_action(start_nav_action_node)

    return ld
