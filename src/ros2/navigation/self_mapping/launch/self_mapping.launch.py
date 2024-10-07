import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='self_mapping',
            executable='self_mapping_node',
            name='self_mapping',
            output='screen'
        )
    ])

def generate_launch_description():
    is_simulation = os.environ["is_simulation"]

    if is_simulation == 'True':
            robot_base_frame = "base_link"
    else:
            robot_base_frame = "robot/base_link"

    robot_base_frame_param = LaunchConfiguration('robot_base_frame_param',  default = robot_base_frame)

    declare_robot_base_frame_cmd = DeclareLaunchArgument(
        'robot_base_frame',
        default_value=robot_base_frame_param,
        description='Robot base frame')

    start_nav_action_node = Node(
            package='self_mapping',
            executable='self_mapping_node',
            name='self_mapping',
            output='screen',
            parameters=[{'robot_base_frame_param': robot_base_frame_param}]
        )


    ld = LaunchDescription()

    ld.add_action(declare_robot_base_frame_cmd)
    ld.add_action(start_nav_action_node)

    return ld