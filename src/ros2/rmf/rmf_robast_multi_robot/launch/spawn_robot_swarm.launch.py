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
    with open("environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    ld = LaunchDescription()
    use_sim_time =True
    for n in range(2):
        namespace="rb"+str(n)
       
        robot_xml = xacro.process_file(
            os.path.join(
                get_package_share_directory("rb_theron_description"),
                "robots",
                environment_yaml["robot"]+".urdf.xacro",
            ),
            mappings={"prefix": environment_yaml["prefix"]},
        ).toxml()
    
        # declare_robot_model_cmd = DeclareLaunchArgument(
        #     "robot_name",
        #     default_value="rb_theron",
        #     description="name of the robot in the simulation",
        # )
   
        start_robot_state_publisher_cmd = Node(
            name="robot_state_publisher",
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_xml}],
            remappings=[('/tf', 'tf'), ('/tf_static','tf_static')],
        )
    

        spawn_robot_cmd = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-name",
                "rb_"+str(n),
                # "-topic",
                # "robot_description",
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
    
        # group = GroupAction([
        #     PushRosNamespace(namespace=namespace), 
        #     # declare_robot_model_cmd, 
        #     start_robot_state_publisher_cmd,
        #     spawn_robot_cmd,
        # ])
        ld.add_action(start_robot_state_publisher_cmd)
        ld.add_action(spawn_robot_cmd)
  
    return ld
