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

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_ros_bridge_yaml = os.path.join(
        get_package_share_directory("tiplu_world"), "config", "gz_ros_bridge.yaml"
    )



    use_sim_time = LaunchConfiguration("use_sim_time")
    world_model = LaunchConfiguration("world_model")
    # robot_name = LaunchConfiguration("robot_name")
    init_x = LaunchConfiguration("init_x", default="-2")
    init_y = LaunchConfiguration("init_y", default="0")
    init_yaw = LaunchConfiguration("init_yaw", default="0")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="4444", description="Top-level namespace"
    )

    declare_world_model_cmd = DeclareLaunchArgument(
        "world_model",
        default_value=os.path.join(
            get_package_share_directory("tiplu_world"), "worlds", "6OG" + ".sdf"
        ),
        description="path to the world model",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="whether to use sim time or not",
    )

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={"gz_args": world_model}.items(),
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([FindPackageShare("rmf_robast"), "/spawn_robot.launch.py"]),
        launch_arguments={
        "init_x": init_x,
        "init_y": init_y,
        "init_yaw": init_yaw,
        "robot_name": "theron1",
        "use_sim_time":use_sim_time,
        }.items(),
    )

    spawn_robot2_cmd = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([FindPackageShare("rmf_robast"), "/spawn_robot.launch.py"]),
        launch_arguments={
        "init_x": init_x,
        "init_y": init_y,
        "init_yaw": init_yaw,
        "robot_name": "theron2",
        "use_sim_time":use_sim_time,
        }.items(),
    )

    

    gz_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": gz_ros_bridge_yaml},
        ],
        output="screen",
    )   
    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_world_model_cmd,
            gz_sim_cmd,
            spawn_robot_cmd,
            spawn_robot2_cmd,
            gz_ros_bridge_cmd
        ]
    )
