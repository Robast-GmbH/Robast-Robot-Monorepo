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
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    ld = LaunchDescription()
    use_sim_time =True
    for n in range(len(environment_yaml["robot_namspaces"])):
        robot_namespace = environment_yaml["robot_namspaces"][n]
        robot_xml = xacro.process_file(
            os.path.join(
                get_package_share_directory("rb_theron_description"),
                "robots",
                environment_yaml["robot"]+".urdf.xacro",
            ),
            mappings={"prefix": environment_yaml["prefix"], "topic_namespace":robot_namespace},
        ).toxml()
    
        declare_robot_model_cmd = DeclareLaunchArgument(
            "robot_name",
            default_value=robot_namespace,
            description="name of the robot in the simulation",
        )
   
        start_robot_state_publisher_cmd = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=robot_namespace,
            parameters=[{'frame_prefix': robot_namespace+'/',"use_sim_time": use_sim_time}, {"robot_description": robot_xml}],
            output="screen",  
        )
    
        joint_state_publisher_node = Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=robot_namespace,
            parameters=[{'frame_prefix': robot_namespace+'/', 'use_sim_time': True}, {"robot_description": robot_xml}],
            output="screen"
        )

        spawn_robot_cmd = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            namespace=robot_namespace,
            arguments=["-name",
                robot_namespace,
                "-topic",
                "robot_description",
                "-z",
                "0.2",
                "-x",
                str(-2+-2*n),
                "-y",
                "0",
                "-Y",
                "0",
            ],
      
        )    

        group = GroupAction([
            declare_robot_model_cmd, 
            start_robot_state_publisher_cmd,
            #joint_state_publisher_node,
            spawn_robot_cmd,
        ])
        ld.add_action(group)
       
  
    return ld
