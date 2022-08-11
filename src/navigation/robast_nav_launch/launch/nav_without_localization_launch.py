import os
import yaml


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    if(os.environ['ROS_DISTRO'] == 'humble'):
        default_bt_xml_filename = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'default_nav_to_pose_bt_xml.xml')
        bt_xml_filename_door_bells = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'navigate_to_pose_w_replanning_and_recovery.xml')

    else:
        default_bt_xml_filename = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'default_nav_to_pose_bt_xml.xml')
        bt_xml_filename_door_bells = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'behavior_trees', os.environ['ROS_DISTRO'], 'default_nav_to_pose_bt_xml.xml')

    nav2_params_yaml = os.path.join(get_package_share_directory('robast_nav_launch'),
                                    'config', 'nav2_params_' + os.environ['ROS_DISTRO']+'.yaml')
    recoveries_params_yaml = os.path.join(get_package_share_directory(
        'robast_nav_launch'), 'config', 'recoveries_params.yaml')

    robast_nav_launch_dir = get_package_share_directory('robast_nav_launch')
    # nav2_params_yaml = LaunchConfiguration('nav2_params_yaml')
    recoveries_launch_file = os.path.join(robast_nav_launch_dir, 'launch', 'recoveries_launch.py')
    robast_nav_interim_goal_dir = get_package_share_directory('robast_nav_interim_goal')
    interim_goal_launch_file = os.path.join(robast_nav_interim_goal_dir, 'launch', 'interim_goal_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_interim_goal = LaunchConfiguration('use_interim_goal')
    use_map_buffer = LaunchConfiguration('use_map_buffer')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'global_frame': namespace+'_map',
        'map_topic': "/map"}

    configured_params = RewrittenYaml(
        source_file=nav2_params_yaml,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
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

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    # {'default_nav_to_pose_bt_xml': default_bt_xml_filename},
                ],
                remappings=remappings,
                # condition=UnlessCondition(use_interim_goal)
            ),

            # Node(
            #     package='nav2_bt_navigator',
            #     executable='bt_navigator',
            #     name='bt_navigator',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[
            #         configured_params,
            #         {'default_nav_to_pose_bt_xml': bt_xml_filename_door_bells},
            #     ],
            #     remappings=remappings,
            #     condition=IfCondition(use_interim_goal)),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            # Node(
            #     package='nav2_velocity_smoother',
            #     executable='velocity_smoother',
            #     name='velocity_smoother',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     remappings=remappings +
            #                [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),


            # Node(
            #     package='theron_fleetmanagement_bridge',
            #     executable='map_buffer',
            #     name='map_buffer',
            #     condition=IfCondition(use_map_buffer)),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings),

            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),

            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[
                    configured_params,
                    # {'default_nav_to_pose_bt_xml': default_bt_xml_filename},
                ],
                remappings=remappings),

            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            # ComposableNode(
            #     package='nav2_velocity_smoother',
            #     plugin='nav2_velocity_smoother::VelocitySmoother',
            #     name='velocity_smoother',
            #     parameters=[configured_params],
            #     remappings=remappings +
            #     [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),


            # Node(
            #     package='theron_fleetmanagement_bridge',
            #     executable='map_buffer',
            #     name='map_buffer',
            #     condition=IfCondition(use_map_buffer)),
        ]
    )

    # launch_robast_recoveries_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(recoveries_launch_file),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'autostart': autostart,
    #         'costmap_namespace': 'recoveries_costmap',
    #         'recoveries_params_file': recoveries_params_yaml,
    #     }.items())

    # launch_interim_goal_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(interim_goal_launch_file),
    #     launch_arguments={'use_sim_time': use_sim_time,
    #                       'autostart': autostart}.items(),
    #     condition=IfCondition(use_interim_goal))

    ld = LaunchDescription()
    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_interim_goal_cmd)
    ld.add_action(declare_use_map_buffer_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    # launches
    # ld.add_action(launch_robast_recoveries_cmd)
    # ld.add_action(launch_interim_goal_cmd)

    return ld
