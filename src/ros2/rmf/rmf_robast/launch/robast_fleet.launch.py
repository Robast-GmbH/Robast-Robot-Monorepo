import yaml
import launch
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = launch.actions.DeclareLaunchArgument(
        "use_sim_time",
        default_value=["true"],
        description="Use the /clock topic for time to sync with simulation",
    )

    failover_mode = launch.actions.DeclareLaunchArgument(
        "failover_mode",
        default_value=["false"],
        description="Enable failover mode for the fleet adapter",
    )

    ## loading configs
    with open("/workspace/src/rmf/rmf_robast/config/fleet_vars.yaml", "r") as stream:
        try:
            fleet_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


    ##rmf core
    common = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ThisLaunchFileDir(), "/common.launch.py"]),
        launch_arguments={
            "use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time"),
            "viz_config_file": [FindPackageShare("rmf_robast"), "/include/6og.rviz"],
            "config_file": [FindPackageShare("rmf_robast"), "/maps/6og/6og.building.yaml"],
            "dashboard_config_file": [
                FindPackageShare("rmf_robast"),
                "/dashboard_config.json",
            ],
        }.items(),
    )
    simulation = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ThisLaunchFileDir(), "/new_simulation.launch.py"]),
    )

    state_aggregator = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                # FindPackageShare('rmf_fleet_adapter'),
                ThisLaunchFileDir(),
                "/robot_state_aggregator.launch.py",
            ]
        ),
        launch_arguments={
            "robot_prefix": fleet_yaml["fleet_name"],
            "fleet_name": fleet_yaml["fleet_name"],
            "use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time"),
            "failover_mode": launch.substitutions.LaunchConfiguration("failover_mode"),
        }.items(),
    )

    ### fleet Adapter
    fleet_adapter_rb_theron = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [FindPackageShare("rmf_robast"), "/rb_theron_adapter.launch.py"]
        ),
        launch_arguments={
            "fleet_name":  fleet_yaml["fleet_name"],
            "use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time"),
            "nav_graph": [
                FindPackageShare("rmf_robast"),
                "/maps/6og/nav_graphs/0.yaml"],
            "config_file": [
                FindPackageShare("rmf_robast"),
                "/config/config.yaml"],
        }.items(),
    )


    ### Free Fleet
    free_fleet_server = launch.actions.IncludeLaunchDescription(
         AnyLaunchDescriptionSource(
            [FindPackageShare("rmf_robast"), "/free_fleet_server.launch.py"]
         ),
    )

    return launch.LaunchDescription(
        [
            use_sim_time,
            failover_mode,
            common,
            simulation,
            # free_fleet_server,
            # fleet_adapter_rb_theron,
            # state_aggregator,
        ]
    )
