from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rmf_visualization_schedule_share_dir = get_package_share_directory(
        "rmf_visualization_schedule"
    )
    rmf_visualization_share_dir = get_package_share_directory("rmf_visualization")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    viz_config_file = LaunchConfiguration(
        "viz_config_file",
        default=f"{rmf_visualization_schedule_share_dir}/config/rmf.rviz",
    )
    config_file = LaunchConfiguration(
        "config_file",
        default="/workspace/src/fleet_management/open_rmf/fleet_adapter_rb_theron/tiplu_Tiplu/tiplu.building.yaml",
        # default="/workspace/src/simulation/rmf_gazebo/maps/tiplu.building.yaml",
    )
    dashboard_config_file = LaunchConfiguration(
        "dashboard_config_file",
        default="/workspace/src/fleet_management/open_rmf/fleet_adapter_rb_theron/dashboard_config.json",
    )
    initial_map = LaunchConfiguration("initial_map", default="Tiplu")
    headless = LaunchConfiguration("headless", default="false")
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

    # Visualizer
    visualization_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            f"{rmf_visualization_share_dir}/visualization.launch.xml"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_name": initial_map,
            "viz_config_file": viz_config_file,
            "headless": headless,
        }.items(),
    )

    # Door Supervisor
    # door_supervisor = Node(
    #     package="rmf_fleet_adapter",
    #     executable="door_supervisor",
    #     parameters=[use_sim_time],
    # )

    # Dispatcher Node
    rmf_task_dispatcher = Node(
        package="rmf_task_ros2",
        executable="rmf_task_dispatcher",
        output="screen",
        parameters=[use_sim_time, bidding_time_window],
    )

    # Dashboard
    dashboard_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            "/workspace/src/fleet_management/open_rmf/fleet_adapter_rb_theron/launch/dashboard.launch.xml"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "dashboard_config_file": dashboard_config_file,
        }.items(),
    )

    # fleet_adapter = Node(
    #     package="fleet_adapter_rb_theron",
    #     executable="fleet_adapter",
    #     name="fleet_adapter",
    #     arguments=[
    #         "--config_file",
    #         "/workspace/src/fleet_management/open_rmf/fleet_adapter_rb_theron/config.yaml",
    #         "--nav_graph",
    #         "/workspace/src/fleet_management/open_rmf/fleet_adapter_rb_theron/tiplu_Tiplu/0.yaml",
    #         "--server_uri",
    #         "http://localhost:8000/_internal",
    #     ],
    #     parameters=[use_sim_time],
    # )

    return LaunchDescription(
        [
            rmf_traffic_schedule_primary,
            rmf_traffic_blockade,
            building_map_server,
            # visualization_launch,
            rmf_task_dispatcher,
            #  dashboard_launch,
            # fleet_adapter,
        ]
    )
