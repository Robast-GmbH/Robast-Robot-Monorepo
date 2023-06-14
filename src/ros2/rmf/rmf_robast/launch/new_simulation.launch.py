import os

import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource



def generate_launch_description():
    with open("environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)
    
    world= launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([FindPackageShare("rmf_robast_multi_robot"),"/launch","/empty_world.launch.py"]),
    )
    fleet=  launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([FindPackageShare("rmf_robast_multi_robot"),"/launch","/spawn_robot_swarm.launch.py"]),
    )
  
    return LaunchDescription(
        [
            world,
            fleet
        ]
    )
