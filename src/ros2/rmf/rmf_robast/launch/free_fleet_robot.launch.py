
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    with open("/workspace/src/navigation/environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


    free_fleet_client_cmd = Node(
            package="free_fleet_client_ros2",
            executable="free_fleet_client_ros2",
            namespace= robot_namespace,
            name="free_fleet_client",
        parameters=[{"fleet_name": "ROBAST_1"},
                    {"robot_frame": robot_namespace+"/base_footprint"},
                    {"robot_name": robot_namespace}, 
                    {"robot_model": "rb_Theron"}, 
                    {"level_name": "L1"},
                    {"dds_domain": 42},
                    {"max_dist_to_first_waypoint": 10.0}, 
                    {"nav2_server_name": "/"+robot_namespace+"/navigate_to_pose"},
                    {"docking_trigger_server_name": ""},],
        output="screen",  
        )  
    
    
    ld = LaunchDescription()
    ld.add_action(free_fleet_client_cmd)
    return ld