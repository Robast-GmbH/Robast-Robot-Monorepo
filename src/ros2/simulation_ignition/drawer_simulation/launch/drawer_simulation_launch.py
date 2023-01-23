
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    drawer_simulation_node = Node(
                package='drawer_simulation', 
                executable='drawer_simulation',
                name="drawer_simulation",
                output='screen') 

    # ld.add_action(joint_state_publisher)
    # ld.add_action(joint_position_controller)   
    ld.add_action(drawer_simulation_node)          
    return ld
