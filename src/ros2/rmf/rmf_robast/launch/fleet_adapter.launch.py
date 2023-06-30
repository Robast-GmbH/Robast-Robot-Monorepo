import launch
import launch_ros.actions


def generate_launch_description():
    use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=['true'],
        description='Use the /clock topic for time to sync with simulation')

    nav_graph = launch.actions.DeclareLaunchArgument(
        'nav_graph',
        default_value=[''],
        description='Nav graph required by fleet adapter')
    
    config_file = launch.actions.DeclareLaunchArgument(
        'config_file',
        default_value=[''],
        description='config for the fleet adapter')

    fleet_adapter_node = launch_ros.actions.Node(
        package='fleet_adapter_ff',
        executable= 'fleet_adapter',
        name=[
            launch.substitutions.LaunchConfiguration('fleet_name'),
            '_fleet_adapter'
        ],
        arguments=[
            # '--use_sim_time ', launch.substitutions.LaunchConfiguration('use_sim_time'),
            '-n', launch.substitutions.LaunchConfiguration('nav_graph'),
            '-c', launch.substitutions.LaunchConfiguration('config_file'),
        ]
    )

    return launch.LaunchDescription([
        use_sim_time,
        nav_graph,
        config_file,
        fleet_adapter_node
    ])