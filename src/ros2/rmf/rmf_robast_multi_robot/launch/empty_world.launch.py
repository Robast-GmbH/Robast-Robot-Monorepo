import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_ros_bridge_yaml = os.path.join(
        get_package_share_directory("tiplu_world"), "config", "gz_ros_bridge.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_model = LaunchConfiguration("world_model")
 
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
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

    gz_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": gz_ros_bridge_yaml},
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_model_cmd)

    # included launches
    ld.add_action(gz_sim_cmd)

    # nodes
    ld.add_action(gz_ros_bridge_cmd)

    return ld
