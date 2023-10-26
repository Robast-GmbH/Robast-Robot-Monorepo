import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld= LaunchDescription()


    config = os.path.join(
        get_package_share_directory('robot_client'),
        'config',
        'params.yaml'
        )

    robot_client= Node(
                    package='robot_client',
                    executable='robot_client',
                    name='robot_client',
                    parameters = [config]
                    )
    ld.add_action(robot_client)
    return ld
