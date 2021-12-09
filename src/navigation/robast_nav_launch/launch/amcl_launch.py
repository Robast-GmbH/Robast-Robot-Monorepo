import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    POSE_INIT_X = os.environ['POSE_INIT_X']
    POSE_INIT_Y = os.environ['POSE_INIT_Y']
    POSE_INIT_Z = os.environ['POSE_INIT_Z']

    if(os.environ['ROS_DISTRO'] == 'galactic'):
        nav2_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'nav2_params_galactic.yaml')
    else:
        nav2_params_yaml = os.path.join(get_package_share_directory('robast_nav_launch'), 'config', 'nav2_params.yaml')

    nav2_localization_params_yaml = os.path.join(get_package_share_directory(
        'robast_nav_launch'), 'config', 'localization_params.yaml')
    map_file = os.path.join(get_package_share_directory('robast_nav_launch'), 'maps', '5OG.yaml')

    print('map_file: {}'.format(map_file))

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Mind the order in which the nodes are started!!!
    lifecycle_nodes = [
        'map_server',
        'amcl',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/map', 'map')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params_yaml,
                        {'yaml_filename': map_file}
                        ],
            remappings=remappings),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                nav2_localization_params_yaml,
                {"initial_pose": {"x": float(POSE_INIT_X), "y": float(POSE_INIT_Y), "yaw": 3.14}},
                {"set_initial_pose": True},
            ],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])
