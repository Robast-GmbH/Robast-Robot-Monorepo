
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource
import xacro

def generate_launch_description():
    with open("environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    init_x = LaunchConfiguration('init_x', default="8.59")
    init_y = LaunchConfiguration('init_y', default="-13.45")
    init_yaw = LaunchConfiguration('init_yaw', default="3.14")
    
    robot_name = LaunchConfiguration('robot_name', default="robot")
    use_sim_time=  LaunchConfiguration('use_sim_time')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='rb_theron',
        description='name of the robot in the simulation')
    robot_xml = xacro.process_file(
        os.path.join(
            get_package_share_directory("rb_theron_description"),
            "robots",
            environment_yaml["robot"] + ".urdf.xacro",
        ),
        mappings={"prefix": environment_yaml["prefix"]},
        ).toxml()
    
    start_robot_state_publisher_cmd = Node(
         package="robot_state_publisher",
         executable="robot_state_publisher",
         name=["robot_state_publisher",robot_name],
         parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_xml}],
         output="screen",
    )

    spawn_robot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-z",
            "0.2",
            "-x",
            init_x,
            "-y",
            init_y,
            "-Y",
            init_yaw,
        ],
        output="screen",
    )

   
    return LaunchDescription(
        [
            declare_robot_name_cmd,
            start_robot_state_publisher_cmd,
            spawn_robot_cmd,
            
        ]
    )