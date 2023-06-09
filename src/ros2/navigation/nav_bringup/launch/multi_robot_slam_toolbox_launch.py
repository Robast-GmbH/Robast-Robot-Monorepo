import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    with open("/workspace/src/navigation/environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    config_directory = environment_yaml["config_directory"]
    is_simulation = environment_yaml["is_simulation"]
    
    nav_bringup_dir = get_package_share_directory("nav_bringup")
    world_posegraph = LaunchConfiguration("world_posegraph")

    if is_simulation:
        world_model = os.path.join(nav_bringup_dir, "maps", "6OG", "6OG_new")
    else:
        world_model = (os.path.join(nav_bringup_dir, "maps", "new6OG", "tiplu_new"),)
    slam_nodes=[]
    for namespace in  environment_yaml["robot_namspaces"]:
        declare_world_model_cmd = DeclareLaunchArgument(
            "world_posegraph",
            default_value=world_model,
            description="path to the world posegraph",
        )
  
        slam_toolbox_params_yaml = os.path.join(
            nav_bringup_dir,
            config_directory,
            "slam",
            "slam_toolbox_params_offline.yaml",
        )
        param_substitutions = {
            "odom_frame": namespace+"/odom",
            "base_frame": namespace+"/base_footprint",
            "scan_topic": "front_laser/scan"
        }
        configured_params = RewrittenYaml(
        source_file=slam_toolbox_params_yaml,
        param_rewrites=param_substitutions,
        convert_types=True)
        
        slam_launch_file = os.path.join(
            nav_bringup_dir, "launch", "multi_robot_slam_toolbox_base_launch.py"
        )

        slam_arguments = {
            "slam_executable": "sync_slam_toolbox_node",
            "slam_params_file": configured_params,
            "slam_posegraph": world_posegraph,
            "slam_mode": "mapping",
            "slam_map_topic": "/map",
            "namespace": namespace,
        }.items()

        launch_slam_base_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file), launch_arguments=slam_arguments
        )
        slam_nodes.append( launch_slam_base_launch)

    ld = LaunchDescription()
    ld.add_action(declare_world_model_cmd)
    for n in slam_nodes:
        ld.add_action(n)

    return ld
