import launch
import launch_ros.actions


def generate_launch_description():
    use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=['false'],
        description='Use the /clock topic for time to sync with simulation')
    viz_config_file = launch.actions.DeclareLaunchArgument(
        'viz_config_file',
        default_value=[
            launch_ros.substitutions.FindPackageShare('dummy_robot_bringup'),
            '/config/rmf.rviz'
        ]
    )
    config_file = launch.actions.DeclareLaunchArgument(
        'config_file',
        description='Building description file required by rmf_building_map_tools')
    dashboard_config_file = launch.actions.DeclareLaunchArgument(
        'dashboard_config_file',
        description='Path to dashboard config for web rmf panel file')
    initial_map = launch.actions.DeclareLaunchArgument(
        'initial_map',
        default_value=['L1'],
        description='Initial map name for the visualizer')
    headless = launch.actions.DeclareLaunchArgument(
        'headless',
        default_value=['false'],
        description='Do not launch rviz and launch gazebo in headless mode')
    bidding_time_window = launch.actions.DeclareLaunchArgument(
        'bidding_time_window',
        default_value=['2.0'],
        description='Time window in seconds for task bidding process')

    # Traffic schedule
    schedule_node = launch_ros.actions.Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_schedule',
        parameters=[{
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            }]
        )

    # Blockade moderator
    blockade_moderator = launch_ros.actions.Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_blockade',
        parameters=[{
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            }]
        )

    # Building map
    building_map = launch_ros.actions.Node(
        package='rmf_building_map_tools',
        executable='building_map_server',
        arguments=[launch.substitutions.LaunchConfiguration('config_file')],
        )

    # # Visualiser TODO:needs to be implemented is not a core feature but would be nice to have
    # visualizer = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.AnyLaunchDescriptionSource(
    #          #[launch_ros.substitutions.FindPackageShare('rmf_visualization'),
    #         [ThisLaunchFileDir(),
    #         '/visualization.launch.py']
    #     ),
    #     launch_arguments=[
    #         ('use_time_time',
    #             launch.substitutions.LaunchConfiguration('use_sim_time')),
    #         ('map_name',
    #             launch.substitutions.LaunchConfiguration('initial_map')),
    #         ('viz_config_file',
    #             launch.substitutions.LaunchConfiguration('viz_config_file')),
    #         ('headless',
    #             launch.substitutions.LaunchConfiguration('headless')),
    #     ]
    # )
    # Dispatcher
    dispatcher = launch_ros.actions.Node(
        package='rmf_task_ros2',
        executable='rmf_task_dispatcher',
        parameters=[{
            'use_time_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            'bidding_time_window':
                launch.substitutions.LaunchConfiguration('bidding_time_window'),
            }]
        )

    return launch.LaunchDescription([
        use_sim_time,
        viz_config_file,
        config_file,
        dashboard_config_file,
        initial_map,
        headless,
        bidding_time_window,
        schedule_node,
        blockade_moderator,
        building_map,
        # visualizer,TODO Torben: needs to be implemented is not a core feature
        # focus but would be nice to have
        dispatcher,
        ])
