
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
    
    # joint parameter for rb_theron
    joint_names_list=["top_drawer_joint", "test"]
    ign_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_joint_topics_list.append("/model/ur10/joint/%s/0/cmd_pos"%joint_name)
    
    # ROS <- IGN, joint state publisher
    joint_state_publisher = Node(
                package='rb_theron_controller_manager', 
                executable='joint_state_publisher',
                name="rb_theron_joint_state_publisher",
                parameters=[
                            {"joint_names": joint_names_list},
                            {"ign_topic": "/world/default/model/ur10/joint_state"},
                           ],
                output='screen')
    
    #  ROS -> IGN,  joint controller
    joint_controller = Node(
                package='rb_theron_controller_manager', 
                executable='joint_position_controller',
                name="rb_theron_joint_position_controller",
                parameters=[
                            {"joint_names": joint_names_list},
                            {"ign_joint_topics": ign_joint_topics_list},
                            {"rate": 200},
                           ],
                output='screen') 

    ld.add_action(joint_state_publisher)
    ld.add_action(joint_controller)          
    return ld
