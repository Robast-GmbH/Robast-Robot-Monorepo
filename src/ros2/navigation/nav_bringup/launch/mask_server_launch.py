import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    is_simulation = os.environ["is_simulation"]

    # Get the launch directory
    costmap_filters_dir = get_package_share_directory("nav_bringup")

    if is_simulation == 'True':
        no_go_mask_params_yaml = os.path.join(
            costmap_filters_dir, "masks", "6OG_sim", "no_go_params.yaml"
        )
        no_go_mask_file_dir = os.path.join(
            costmap_filters_dir, "masks", "6OG_sim", "sim_world1_keepout.yaml"
        )
    else:
        no_go_mask_params_yaml = os.path.join(
            costmap_filters_dir, "masks", "RL_Tiplu", "keepout_params.yaml"
        )
        no_go_mask_file_dir = os.path.join(
            costmap_filters_dir, "masks", "RL_Tiplu", "6_OG_24_5_keepout.yaml"
        )

    # Create our own temporary YAML files that include substitutions
    lifecycle_nodes = ["filter_mask_server", "costmap_filter_info_server"]

    # Parameters
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    no_go_mask_params = LaunchConfiguration("no_go_mask_params")
    no_go_mask_file = LaunchConfiguration("no_go_mask_file")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_no_go_mask_params_yaml_cmd = DeclareLaunchArgument(
        "no_go_mask_params",
        default_value=no_go_mask_params_yaml,
        description="Full path to the ROS 2 parameters file to use",
    )

    declare_no_go_mask_file_cmd = DeclareLaunchArgument(
        "no_go_mask_file",
        default_value=no_go_mask_file_dir,
        description="Full path to filter mask yaml file to load",
    )

    # Make re-written yaml
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "yaml_filename": no_go_mask_file,
    }

    configured_params = RewrittenYaml(
        source_file=no_go_mask_params,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap_filters",
        namespace=namespace,
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    start_map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        namespace=namespace,
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[configured_params],
    )

    start_costmap_filter_info_server_cmd = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        namespace=namespace,
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[configured_params],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_no_go_mask_params_yaml_cmd)
    ld.add_action(declare_no_go_mask_file_cmd)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)

    return ld
