import launch
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    fleet_name = launch.actions.DeclareLaunchArgument(
        'fleet_name',
        default_value=['rb_theron_fleet'],
        description='The name that will be published in the outgoing fleet state')
    use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=['True'],
        description='Use the /clock topic for time to sync with simulation')
    nav_graph = launch.actions.DeclareLaunchArgument(
        'nav_graph',
        description='Nav graph required by fleet adapter')

    fleet_adapter = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ThisLaunchFileDir(),
                                    '/lf_fleet_adapter.launch.xml']),
                                    launch_arguments={
                                        'fleet_name':
                                            launch.substitutions.LaunchConfiguration('fleet_name'),
                                        'control_type': 'full_control',
                                        'nav_graph':
                                            launch.substitutions.LaunchConfiguration('nav_graph'),
                                        'linear_velocity': '0.5',
                                        'angular_velocity': '0.6',
                                        'linear_acceleration': '0.75',
                                        'angular_acceleration': '2.0',
                                        'footprint_radius': '0.3',
                                        'vicinity_radius': '1.0',
                                        'use_sim_time':
                                            launch.substitutions.LaunchConfiguration('use_sim_time'),
                                        'delay_threshold': '15.0',
                                        'retry_wait': '10.0',
                                        'discovery_timeout': '60.0',
                                        'perform_deliveries': 'true',
                                        'perform_loop': 'true',
                                        'perform_cleaning': 'false',
                                        'battery_voltage': '12.0',
                                        'battery_capacity': '24.0',
                                        'battery_charging_current': '5.0',
                                        'mass': '20.0',
                                        'inertia': '10.0',
                                        'friction_coefficient': '0.22',
                                        'ambient_power_drain': '20.0',
                                        'tool_power_drain': '0.0',
                                        'drain_battery': 'true',
                                        'recharge_threshold': '0.1'
                                    }.items())

    return launch.LaunchDescription([
        fleet_name,
        use_sim_time,
        nav_graph,
        fleet_adapter,
        ])
