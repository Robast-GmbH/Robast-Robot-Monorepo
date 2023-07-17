import launch
import launch_ros.actions


def generate_launch_description():
    use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=['false'],
        description='Use the /clock topic for time to sync with simulation')
    rate = launch.actions.DeclareLaunchArgument(
        'rate',
        default_value=['10'],
        description='Schedule visualisation update rate')
    map_name = launch.actions.DeclareLaunchArgument(
        'map_name',
        default_value=['B1'],
        description='Name of the map within the world')
    viz_config_file = launch.actions.DeclareLaunchArgument(
        'viz_config_file',
        default_value=[
            launch_ros.substitutions.FindPackageShare('rmf_visualization_schedule'),
            '/config/rmf.rviz'
        ]
    )
    display_names = launch.actions.DeclareLaunchArgument(
        'display_names',
        default_value=['true'])
    websocket_port = launch.actions.DeclareLaunchArgument(
        'websocket_port',
        default_value=['8086'])
    headless = launch.actions.DeclareLaunchArgument(
        'headless',
        default_value=['false'],
        description='Do not launch rviz')

    rmf_viz_schedule_node = launch_ros.actions.Node(
        package='rmf_visualization_schedule',
        executable='schedule_visualizer_node',
        arguments=[
            '-r', launch.substitutions.LaunchConfiguration('rate'),
            '-m', launch.substitutions.LaunchConfiguration('map_name'),
            '-p', launch.substitutions.LaunchConfiguration('websocket_port'),
            '--history', '50'
        ],
        parameters=[{
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
        }]
    )

    rmf_viz_fleet_states_node = launch_ros.actions.Node(
        package='rmf_visualization_fleet_states',
        executable='fleetstates_visualizer_node',
        arguments=['-m', launch.substitutions.LaunchConfiguration('map_name')],
        parameters=[{
            'display_names':
                launch.substitutions.LaunchConfiguration('display_names'),
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            }]
        )

    rmf_viz_building_systems_node = launch_ros.actions.Node(
        package='rmf_visualization_building_systems',
        executable='rmf_visualization_building_systems',
        arguments=['-m', launch.substitutions.LaunchConfiguration('map_name')],
        parameters=[{
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            }]
        )

    rviz = launch.actions.ExecuteProcess(
        cmd=['rviz2', '-d',
             launch.substitutions.LaunchConfiguration('viz_config_file')])

    return launch.LaunchDescription([
        use_sim_time,
        rate,
        map_name,
        viz_config_file,
        display_names,
        websocket_port,
        headless,
        rmf_viz_schedule_node,
        rmf_viz_fleet_states_node,
        rmf_viz_building_systems_node,
        rviz
    ])
