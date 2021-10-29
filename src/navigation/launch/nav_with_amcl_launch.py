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

    nav2_params_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'nav2_params.yaml')
    nav2_localization_params_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'localization_params.yaml')
    default_bt_xml_filename = os.path.join(get_package_share_directory('navigation'), 'behavior_trees', os.environ['ROS_DISTRO'], 'navigate_w_replanning_and_recovery.xml')
    map_file = os.path.join(get_package_share_directory('navigation'), 'maps', '5OG_map.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('navigation'), 'config', 'nav2_default_view.rviz')


    print('map_file: {}'.format(map_file))

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Mind the order in which the nodes are started!!!
    lifecycle_nodes = [
                        'map_server',
                        'amcl',
                        'controller_server',
                        'planner_server',
                        'recoveries_server',
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
                {"initial_pose": {"x": 8.59, "y": -13.45, "yaw": 3.14}},
                {"set_initial_pose": True},
            ],
            remappings=remappings),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_yaml],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_yaml],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_yaml],
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
