import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():


    config_directory = os.environ['config_directory']
    is_simulation = os.environ["is_simulation"]

    namespace = LaunchConfiguration("namespace")
    nav_bringup_dir = get_package_share_directory("nav_bringup")
    world_posegraph = LaunchConfiguration("world_posegraph")

    if is_simulation == 'True':
        world_model = os.path.join(nav_bringup_dir, "maps", "6OG", "6OG_new")
    else:
        world_model = os.path.join(nav_bringup_dir, "maps", "6OG_Tiplu_July", "RL_Tiplu_6")

    declare_world_model_cmd = DeclareLaunchArgument(
        "world_posegraph",
        default_value=world_model,
        description="path to the world posegraph",
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top level namespace"
    )
    slam_toolbox_params_yaml = os.path.join(
        nav_bringup_dir,
        config_directory,
        "slam",
        "slam_toolbox_params_offline.yaml",
    )
    slam_launch_file = os.path.join(
        nav_bringup_dir, "launch", "slam_toolbox_base_launch.py"
    )

    slam_arguments = {
        "slam_executable": "localization_slam_toolbox_node",
        "slam_params_file": slam_toolbox_params_yaml,
        "slam_posegraph": world_posegraph,
        "slam_mode": "mapping",
        "slam_map_topic": "/map",
        "namespace": namespace,
    }.items()

    launch_slam_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file), launch_arguments=slam_arguments
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_model_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(launch_slam_base_launch)

    return ld
