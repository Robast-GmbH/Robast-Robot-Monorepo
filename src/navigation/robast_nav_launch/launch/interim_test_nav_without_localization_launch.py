import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    if(os.environ['ROS_DISTRO'] == 'galactic'):
        default_bt_xml_filename = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'navigate_through_poses_w_replanning_and_recovery.xml')
        nav2_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'nav2_params_galactic.yaml')
    else:
        default_bt_xml_filename = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'interim_test_subtrees.xml')
        nav2_params_yaml = os.path.join(get_package_share_directory('robast_nav_launch'), 'config', 'nav2_params.yaml')

    interim_goals_yaml = os.path.join(get_package_share_directory(
        'robast_nav_interim_goal'), 'config', 'interim_goals.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'interim_goal_selector',
        'bt_navigator',
        'waypoint_follower',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

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
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_yaml,
                        {"map_topic": '/map'}
                        ],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_yaml,
                        {"map_topic": '/map'}],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_yaml],
            remappings=remappings),

        Node(
            package='robast_nav_interim_goal',
            executable='interim_goal_selector',
            name='interim_goal_selector',
            output='screen',
            parameters=[{'interim_goals_yaml': interim_goals_yaml},
                        {'k_nearest_neighbors': 3},
                        {'max_interim_dist_to_path': 0.6}],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_params_yaml,
                {'default_bt_xml_filename': default_bt_xml_filename},
            ],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_yaml],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

    ])
