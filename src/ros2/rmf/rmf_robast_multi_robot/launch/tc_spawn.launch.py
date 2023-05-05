#!/usr/bin/env python3

import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import TimerAction
from launch_ros.descriptions import ParameterValue 

def launch_setup(context, *args, **kwargs):
    with open("/workspace/src/navigation/environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    x_spawn = LaunchConfiguration('x_spawn').perform(context)
    y_spawn = LaunchConfiguration('y_spawn').perform(context)
    entity_name = LaunchConfiguration('entity_name').perform(context)
    use_sim_time =True

    print("###############################################################")
    print("SPAWN MULTI Robot="+str(entity_name)+",["+str(x_spawn)+","+str(y_spawn)+"]")
    print("###############################################################")


    # ROBOT STATE PUBLISHER
    ####### DATA INPUT ##########

    robot_xml = xacro.process_file(
            os.path.join(
                get_package_share_directory("rb_theron_description"),
                "robots",
                environment_yaml["robot"]+".urdf.xacro",
            ),
            mappings={"prefix": environment_yaml["prefix"]},
        ).toxml()
    
    xacro_file = 'rb_theron.urdf.xacro'

    
    package_description = "rb_theron_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "robots", xacro_file)
    
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name,
        parameters=[{'frame_prefix': entity_name+'/', 'use_sim_time': True, 'robot_description': ParameterValue(Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name]), value_type=str)}],
        output="screen"
    )

    # Spawn ROBOT Set Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        namespace=entity_name,
        output='screen',
        arguments=['-name',
                   entity_name,
                   '-x', x_spawn, '-y', y_spawn,
                   '-topic', 'robot_description',
                   '-timeout', '120.0'
                   ]
    )

    return [robot_state_publisher_node, start_gazebo_ros_spawner_cmd]


def generate_launch_description(): 

    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='1.0')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='2.0')
    entity_name_arg = DeclareLaunchArgument('entity_name', default_value='barista_1')
    

    return LaunchDescription([
        x_spawn_arg,
        y_spawn_arg, 
        entity_name_arg,
        OpaqueFunction(function = launch_setup)
        ])
