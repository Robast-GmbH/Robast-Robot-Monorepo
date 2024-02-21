import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    gazebo_launch_file = os.path.join(
        get_package_share_directory("tiplu_world"), "launch", "tiplu_world_launch.py"
    )

    launch_arguments = {
        "model_position_joint": "True",
    }.items()

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file), launch_arguments=launch_arguments
    )

    ld = LaunchDescription()
    ld.add_action(launch_gazebo)

    return ld
