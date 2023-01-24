
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
    joint_names_list=["drawer_1_joint", "drawer_2_joint", "drawer_3_joint", "drawer_4_joint", "drawer_5_joint"]
    gz_joint_topics_list=[]
    for joint_name in joint_names_list:
        gz_joint_topics_list.append("/model/rb_theron/joint/%s/0/cmd_pos"%joint_name)
    
    #  ROS -> IGN,  joint position controller
    # We can either use the joint position controller or the joint trajectory controller.
    # In case we want moveit to plan and execute the motion, we have to go with the trajectory controller
    joint_position_controller = Node(
                package='rb_theron_controller_manager', 
                executable='joint_position_controller',
                name="rb_theron_joint_position_controller",
                parameters=[
                            {"joint_names": joint_names_list},
                            {"gz_joint_topics": gz_joint_topics_list},
                            {"rate": 200},
                           ],
                output='screen')
    
    joint_trajectory_controller = Node(
                package='rb_theron_controller_manager', 
                executable='joint_trajectory_controller',
                name="rb_theron_joint_trajectory_controller",
                parameters=[
                            {"joint_names": joint_names_list},
                            {"gz_joint_topics": gz_joint_topics_list},
                            {"rate": 200},
                           ],
                output='screen') 

    # ld.add_action(joint_state_publisher)
    ld.add_action(joint_position_controller)   
    ld.add_action(joint_trajectory_controller)          
    return ld
