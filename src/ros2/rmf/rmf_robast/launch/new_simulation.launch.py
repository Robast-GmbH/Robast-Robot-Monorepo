
import yaml
import launch
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():

    world = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([FindPackageShare("rmf_robast_multi_robot"),
                                    "/launch",
                                    "/empty_world.launch.py"]),
    )
    
    fleet = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([FindPackageShare("rmf_robast_multi_robot"),
                                    "/launch",
                                    "/spawn_robot_swarm.launch.py"]),
    )

    return LaunchDescription(
        [
            world,
            fleet
        ]
    )
