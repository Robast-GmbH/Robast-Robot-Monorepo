import launch
import os
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    map_package = launch.actions.DeclareLaunchArgument(
        "map_package",
        default_value=["rmf_robast"],
        description="Name of the map package",
    )

    use_crowdsim = launch.actions.DeclareLaunchArgument(
        "use_crowdsim",
        default_value=["false"],
        description="Use Crowdsim to simulate crowds",
    )

    plugin_path = launch.actions.SetLaunchConfiguration(
        "plugin_path",
        [
            FindPackagePrefix("rmf_robot_sim_gz_plugins"),
            "/lib:",
            FindPackagePrefix("rmf_building_sim_gz_plugins"),
            "/lib:/usr/share/gazebo-",
            launch.substitutions.LaunchConfiguration("gazebo_version"),
        ],
    )

    menge_resource_path = launch.actions.SetLaunchConfiguration(
        "menge_resource_path", ""
    )

    world_model = LaunchConfiguration("world_model")
    declare_world_model_cmd = DeclareLaunchArgument(
        "world_model",
         default_value=os.path.join(
            get_package_share_directory("tiplu_world"), "worlds", "6OG" + ".sdf"
        ),
        description="path to the world model",
    )
    declare_robot_model_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="rb_theron",
        description="name of the robot in the simulation",
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={"gz_args": ["-r ", world_model, ],
                          "gz_version": "7",
                          }.items(),
    )
    return launch.LaunchDescription(
        [
            map_package,
            use_crowdsim,
            declare_robot_model_cmd,
            declare_world_model_cmd,
            plugin_path,
            menge_resource_path,
            ignition,
        ]
    )
