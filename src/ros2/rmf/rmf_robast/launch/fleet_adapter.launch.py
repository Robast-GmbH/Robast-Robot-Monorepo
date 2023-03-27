import launch
import launch_ros.actions


def generate_launch_description():
    use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=['true'],
        description='Use the /clock topic for time to sync with simulation')
    control_type = launch.actions.DeclareLaunchArgument(
        'control_type',
        description='Fleet adapter type: full_control or read_only')
    fleet_name = launch.actions.DeclareLaunchArgument(
        'fleet_name',
        default_value=['rb_theron'],
        description='Name of the fleet that this adapter will interface with')
    nav_graph_file = launch.actions.DeclareLaunchArgument(
        'nav_graph_file',
        default_value=[''],
        description='Nav graph required by fleet adapter')
    linear_velocity = launch.actions.DeclareLaunchArgument(
        'linear_velocity',
        description='The nominal linear velocity of the vehicles in this fleet')
    angular_velocity = launch.actions.DeclareLaunchArgument(
        'angular_velocity',
        description='The nominal angular velocity of the vehicles in this fleet')
    linear_acceleration = launch.actions.DeclareLaunchArgument(
        'linear_acceleration',
        description='The nominal linear acceleration of the vehicles in this fleet')
    angular_acceleration = launch.actions.DeclareLaunchArgument(
        'angular_acceleration',
        description='The nominal angular acceleration of the vehicles in this fleet')
    footprint_radius = launch.actions.DeclareLaunchArgument(
        'footprint_radius',
        description='The radius of the footprint of the vehicles in this fleet')
    vicinity_radius = launch.actions.DeclareLaunchArgument(
        'vicinity_radius',
        description='The radius of the personal vicinity of the vehicles in this fleet')
    perform_deliveries = launch.actions.DeclareLaunchArgument(
        'perform_deliveries',
        default_value=['false'],
        description='Whether this fleet adapter can perform deliveries')
    delay_threshold = launch.actions.DeclareLaunchArgument(
        'delay_threshold',
        default_value=['10.0'],
        description='How long to wait before replanning')
    disable_delay_threshold = launch.actions.DeclareLaunchArgument(
        'disable_delay_threshold',
        default_value=['false'],
        description='Disable the delay_threshold behavior')
    retry_wait = launch.actions.DeclareLaunchArgument(
        'retry_wait',
        default_value=['10.0'],
        description='How long a retry should wait before starting')
    discovery_timeout = launch.actions.DeclareLaunchArgument(
        'discovery_timeout',
        default_value=['10.0'],
        description='How long to wait on discovery before giving up')
    reversible = launch.actions.DeclareLaunchArgument(
        'reversible',
        default_value=['true'],
        description='Can the robot drive backwards')
    perform_loop = launch.actions.DeclareLaunchArgument(
        'perform_loop',
        default_value=['false'],
        description='Whether this fleet adapter can perform loops')
    perform_cleaning = launch.actions.DeclareLaunchArgument(
        'perform_cleaning',
        default_value=['false'],
        description='Whether this fleet adapter can perform cleaning')
    battery_voltage = launch.actions.DeclareLaunchArgument(
        'battery_voltage',
        description='The nominal voltage(V) of the battery in the vehicles in this fleet')
    battery_capacity = launch.actions.DeclareLaunchArgument(
        'battery_capacity',
        description='The nominal capacity(Ah) of the battery in the vehicles in this fleet')
    battery_charging_current = launch.actions.DeclareLaunchArgument(
        'battery_charging_current',
        description='The nominal charging current(A) of the battery in the vehicles in this fleet')
    mass = launch.actions.DeclareLaunchArgument(
        'mass',
        description='The mass(kg) of the vehicles in this fleet')
    inertia = launch.actions.DeclareLaunchArgument(
        'inertia',
        description='The inertia(kg.m^2) of the vehicles in this fleet')
    friction_coefficient = launch.actions.DeclareLaunchArgument(
        'friction_coefficient',
        description='The friction coefficient of the vehicles in this fleet')
    ambient_power_drain = launch.actions.DeclareLaunchArgument(
        'ambient_power_drain',
        description='The power rating(W) of ambient devices (processors, sensors, etc.) of the vehicles in this fleet')
    tool_power_drain = launch.actions.DeclareLaunchArgument(
        'tool_power_drain',
        description='The power rating(W) of special tools (vaccuum, cleaning systems, etc.) of the vehicles in this fleet')
    drain_battery = launch.actions.DeclareLaunchArgument(
        'drain_battery',
        default_value=['false'],
        description='Whether battery drain should be considered while assigning tasks to vechiles in this fleet')
    recharge_threshold = launch.actions.DeclareLaunchArgument(
        'recharge_threshold',
        default_value=['0.2'],
        description='The fraction of total battery capacity below which the robot must return to its charger')
    recharge_soc = launch.actions.DeclareLaunchArgument(
        'recharge_soc',
        default_value=['1.0'],
        description='The fraction of total battery capacity to which the robot should be charged')
    experimental_lift_watchdog_service = launch.actions.DeclareLaunchArgument(
        'experimental_lift_watchdog_service',
        default_value=[''],
        description='(Experimental) The name of a service to check whether a robot can enter a lift')

    fleet_adapter_node = launch_ros.actions.Node(
        package='rmf_fleet_adapter',
        executable=launch.substitutions.LaunchConfiguration('control_type'),
        name=[
            launch.substitutions.LaunchConfiguration('fleet_name'),
            '_fleet_adapter'
        ],
        parameters=[{
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            'fleet_name':
                launch.substitutions.LaunchConfiguration('fleet_name'),
            'nav_graph_file':
                launch.substitutions.LaunchConfiguration('nav_graph_file'),
            'linear_velocity':
                launch.substitutions.LaunchConfiguration('linear_velocity'),
            'angular_velocity':
                launch.substitutions.LaunchConfiguration('angular_velocity'),
            'linear_acceleration':
                launch.substitutions.LaunchConfiguration('linear_acceleration'),
            'angular_acceleration':
                launch.substitutions.LaunchConfiguration('angular_acceleration'),
            'footprint_radius':
                launch.substitutions.LaunchConfiguration('footprint_radius'),
            'vicinity_radius':
                launch.substitutions.LaunchConfiguration('vicinity_radius'),
            'perform_deliveries':
                launch.substitutions.LaunchConfiguration('perform_deliveries'),
            'perform_loop':
                launch.substitutions.LaunchConfiguration('perform_loop'),
            'perform_cleaning':
                launch.substitutions.LaunchConfiguration('perform_cleaning'),
            'delay_threshold':
                launch.substitutions.LaunchConfiguration('delay_threshold'),
            'disable_delay_threshold':
                launch.substitutions.LaunchConfiguration('disable_delay_threshold'),
            'retry_wait':
                launch.substitutions.LaunchConfiguration('retry_wait'),
            'discovery_timeout':
                launch.substitutions.LaunchConfiguration('discovery_timeout'),
            'reversible':
                launch.substitutions.LaunchConfiguration('reversible'),
            'battery_voltage':
                launch.substitutions.LaunchConfiguration('battery_voltage'),
            'battery_capacity':
                launch.substitutions.LaunchConfiguration('battery_capacity'),
            'battery_charging_current':
                launch.substitutions.LaunchConfiguration('battery_charging_current'),
            'mass':
                launch.substitutions.LaunchConfiguration('mass'),
            'inertia':
                launch.substitutions.LaunchConfiguration('inertia'),
            'friction_coefficient':
                launch.substitutions.LaunchConfiguration('friction_coefficient'),
            'ambient_power_drain':
                launch.substitutions.LaunchConfiguration('ambient_power_drain'),
            'tool_power_drain':
                launch.substitutions.LaunchConfiguration('tool_power_drain'),
            'drain_battery':
                launch.substitutions.LaunchConfiguration('drain_battery'),
            'recharge_threshold':
                launch.substitutions.LaunchConfiguration('recharge_threshold'),
            'recharge_soc':
                launch.substitutions.LaunchConfiguration('recharge_soc'),
            'experimental_lift_watchdog_service':
                launch.substitutions.LaunchConfiguration('experimental_lift_watchdog_service'),
        }]
    )

    return launch.LaunchDescription([
        use_sim_time,
        control_type,
        fleet_name,
        nav_graph_file,
        linear_velocity,
        angular_velocity,
        linear_acceleration,
        angular_acceleration,
        footprint_radius,
        vicinity_radius,
        perform_deliveries,
        delay_threshold,
        disable_delay_threshold,
        retry_wait,
        discovery_timeout,
        reversible,
        perform_loop,
        perform_cleaning,
        battery_voltage,
        battery_capacity,
        battery_charging_current,
        mass,
        inertia,
        friction_coefficient,
        ambient_power_drain,
        tool_power_drain,
        drain_battery,
        recharge_threshold,
        recharge_soc,
        experimental_lift_watchdog_service,
        fleet_adapter_node
    ])