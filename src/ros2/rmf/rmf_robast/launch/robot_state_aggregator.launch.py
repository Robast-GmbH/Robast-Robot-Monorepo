import launch
import launch_ros.actions


def generate_launch_description():
    failover_mode = launch.actions.DeclareLaunchArgument(
        'failover_mode',
        default_value=['false'],
        description='Enable failover mode for the fleet adapter')
    use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=['false'],
        description='Use the /clock topic for time to sync with simulation')
    robot_prefix = launch.actions.DeclareLaunchArgument(
        'robot_prefix',
        default_value=[''],
        description='The prefix that this aggregator should look for in the incoming robot name')
    fleet_name = launch.actions.DeclareLaunchArgument(
        'fleet_name',
        description='The name that will be published in the outgoing fleet state')

    robot_state_aggregator_node = launch_ros.actions.Node(
        package='rmf_fleet_adapter',
        executable='robot_state_aggregator',
        name=[
            launch.substitutions.LaunchConfiguration('fleet_name'),
            '_state_aggregator'
        ],
        parameters=[{
            'robot_prefix':
                launch.substitutions.LaunchConfiguration('robot_prefix'),
            'fleet_name':
                launch.substitutions.LaunchConfiguration('fleet_name'),
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            'active_node': True,
            'failover_mode': False,
            }]
        )

    return launch.LaunchDescription([
        failover_mode,
        use_sim_time,
        robot_prefix,
        fleet_name,
        robot_state_aggregator_node,
    ])
