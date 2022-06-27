import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    if(os.environ['ROS_DISTRO'] == 'galactic'):
        default_bt_xml_filename = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'navigate_to_pose_w_replanning_and_recovery.xml')
        bt_xml_filename_door_bells = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'nav_bt_with_door_bells.xml')
        nav2_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'nav2_params_galactic_robot.yaml')
        recoveries_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'recoveries_params.yaml')
    else:
        default_bt_xml_filename = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'navigate_w_replanning_and_recovery.xml')
        bt_xml_filename_door_bells = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'nav_bt_with_door_bells.xml')
        nav2_params_yaml = os.path.join(get_package_share_directory('robast_nav_launch'), 'config', 'nav2_params.yaml')
        recoveries_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'recoveries_params.yaml')

    robast_nav_launch_dir = get_package_share_directory('robast_nav_launch')
    recoveries_launch_file = os.path.join(robast_nav_launch_dir, 'launch', 'recoveries_launch.py')
    robast_nav_interim_goal_dir = get_package_share_directory('robast_nav_interim_goal')
    interim_goal_launch_file = os.path.join(robast_nav_interim_goal_dir, 'launch', 'interim_goal_launch.py')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_interim_goal = LaunchConfiguration('use_interim_goal')
    use_map_buffer = LaunchConfiguration('use_map_buffer')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    remappings_movement = [('/cmd_vel', 'robot/robotnik_base_control/cmd_vel'),
                           ('/odom', 'robot/robotnik_base_control/odom'),
                           ('/robot/tf', 'tf'),
                           ('/robot/tf_static', 'tf_static')
                           ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/robot/tf', 'tf'),
                  ('/robot/tf_static', 'tf_static')]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_interim_goal_cmd = DeclareLaunchArgument(
        'use_interim_goal',
        default_value='false',
        description='Use the interim goal navigation if true')

    declare_use_map_buffer_cmd = DeclareLaunchArgument(
        'use_map_buffer',
        default_value='false',
        description='Use a map buffer if true. Mainly required for SLAM because frequent map updates can disturb navigation')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    print('world_file_path : {}'.format(nav2_params_yaml))

    start_controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        namespace=namespace,
        parameters=[nav2_params_yaml,
                    {'use_sim_time': use_sim_time},
                    {"map_topic": '/robot/map'}
                    ],
        remappings=remappings_movement)

    start_planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        namespace=namespace,
        parameters=[nav2_params_yaml,
                    {'use_sim_time': use_sim_time},
                    {"map_topic": '/robot/map'}],
        remappings=remappings_movement)

    start_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        namespace=namespace,
        parameters=[
            nav2_params_yaml,
            {'use_sim_time': use_sim_time},
            {'default_nav_to_pose_bt_xml': default_bt_xml_filename},
        ],
        remappings=remappings_movement,
        condition=UnlessCondition(use_interim_goal))

    start_bt_navigator_with_interim_goal_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        namespace=namespace,
        parameters=[
            nav2_params_yaml,
            {'use_sim_time': use_sim_time},
            {'default_nav_to_pose_bt_xml': bt_xml_filename_door_bells},
        ],
        remappings=remappings_movement,
        condition=IfCondition(use_interim_goal))

    start_waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_yaml],
        remappings=remappings_movement)

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    start_map_buffer_cmd = Node(
        package='theron_fleetmanagement_bridge',
        executable='map_buffer',
        name='map_buffer',
        namespace=namespace,
        condition=IfCondition(use_map_buffer))

    launch_robast_recoveries_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recoveries_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'costmap_namespace': 'recoveries_costmap',
            'recoveries_params_file': recoveries_params_yaml,
        }.items())

    launch_interim_goal_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(interim_goal_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'autostart': autostart}.items(),
        condition=IfCondition(use_interim_goal))

    ld = LaunchDescription()
    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_interim_goal_cmd)
    ld.add_action(declare_use_map_buffer_cmd)
    ld.add_action(declare_namespace_cmd)

    # nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_controller_cmd)
    ld.add_action(start_planner_cmd)
    ld.add_action(start_bt_navigator_cmd)
    ld.add_action(start_bt_navigator_with_interim_goal_cmd)
    ld.add_action(start_waypoint_follower_cmd)
    ld.add_action(start_map_buffer_cmd)

    # launches
    ld.add_action(launch_robast_recoveries_cmd)
    ld.add_action(launch_interim_goal_cmd)

    return ld
