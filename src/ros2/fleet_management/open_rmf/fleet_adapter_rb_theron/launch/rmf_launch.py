from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    config_file = LaunchConfiguration(
        "config_file",
        default="/workspace/src/open_rmf/fleet_adapter_rb_theron/maps/6OG_bp/tiplu.building.yaml",
    )

    bidding_time_window = LaunchConfiguration("bidding_time_window", default="2.0")

    # Traffic Schedule
    rmf_traffic_schedule_primary = Node(
        package="rmf_traffic_ros2",
        executable="rmf_traffic_schedule",
        output="both",
        name="rmf_traffic_schedule_primary",
        parameters=[use_sim_time],
    )

    # Blockade Moderator
    rmf_traffic_blockade = Node(
        package="rmf_traffic_ros2",
        executable="rmf_traffic_blockade",
        output="both",
        parameters=[use_sim_time],
    )

    # Building Map
    building_map_server = Node(
        package="rmf_building_map_tools",
        executable="building_map_server",
        arguments=[config_file],
        parameters=[use_sim_time],
    )

    # Dispatcher Node
    rmf_task_dispatcher = Node(
        package="rmf_task_ros2",
        executable="rmf_task_dispatcher",
        output="screen",
        parameters=[use_sim_time, bidding_time_window],
    )

    door_supervisor = Node(
        package="rmf_fleet_adapter",
        executable="door_supervisor",
        parameters=[use_sim_time],
    )

    fleet_adapter = Node(
        package="fleet_adapter_rb_theron",
        executable="fleet_adapter",
        name="fleet_adapter",
        arguments=[
            "--config_file",
            "/workspace/src/open_rmf/fleet_adapter_rb_theron/config.yaml",
            "--nav_graph",
            "/workspace/src/open_rmf/fleet_adapter_rb_theron/maps/6OG_bp/0.yaml",
        ],
        parameters=[use_sim_time],
    )

    return LaunchDescription(
        [
            rmf_traffic_schedule_primary,
            rmf_traffic_blockade,
            building_map_server,
            rmf_task_dispatcher,
            door_supervisor,
            fleet_adapter,
        ]
    )
