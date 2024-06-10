import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_studio_utils_py.system_config import SystemConfigParser


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    optional_feature_setting = system_config_parser.get_optional_feature_configs()
    use_gui = optional_feature_setting.get("gazebo_gui", False)
    gazebo_world_name = optional_feature_setting.get("gazebo_world_name", "6OG.sdf")
    gazebo_world_model_package = optional_feature_setting.get("gazebo_world_model_package", "tiplu_world")
    
    world_model = os.path.join(get_package_share_directory(gazebo_world_model_package), "worlds", gazebo_world_name)

    gazebo_launch_file = os.path.join(
        get_package_share_directory("tiplu_world"), "launch", "tiplu_world_launch.py"
    )

    launch_arguments = {
        "model_position_joint": "prismatic",
        "world_model": world_model,
    }

    if not use_gui:
        launch_arguments["headless"] = "-s --headless-rendering"

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file), launch_arguments=launch_arguments.items()
    )

    ld = LaunchDescription()
    ld.add_action(launch_gazebo)

    return ld
