import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_studio_utils_py.system_config import SystemConfigParser

def generate_launch_description():
    system_config_parser = SystemConfigParser()
    optional_feature_setting = system_config_parser.get_optional_feature_configs()
    ompl_planning_file = optional_feature_setting.get("ompl_planning_file", "ompl_iron.yaml")
    launch_moveit_group = optional_feature_setting.get("launch_moveit_group", False)
    launch_rviz = optional_feature_setting.get("launch_rviz", True)
    launch_robot_state_publisher = optional_feature_setting.get("launch_robot_state_publisher", True)

    moveit_rviz_simulation_launch_file = os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_config"), "launch", "moveit_rviz_simulation_launch.py"
    )

    launch_arguments = {}
    launch_arguments["ompl_planning_file"] = ompl_planning_file
    launch_arguments["launch_moveit_group"] = str(launch_moveit_group).lower()
    launch_arguments["launch_rviz"] = str(launch_rviz).lower()
    launch_arguments["launch_robot_state_publisher"] = str(launch_robot_state_publisher).lower()

    launch_moveit_rviz_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_rviz_simulation_launch_file), launch_arguments=launch_arguments.items()
    )

    ld = LaunchDescription()
    ld.add_action(launch_moveit_rviz_simulation)

    return ld
