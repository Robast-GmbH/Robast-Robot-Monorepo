from launch import LaunchDescription
from launch_ros.actions import Node
from ros_gazebo_sim import GazeboLaunch

def generate_launch_description():
    return LaunchDescription([
        GazeboLaunch(
            worlds=['empty'],
            gui=True,
            gazebo_params=[
                {
                    'name': 'use_sim_time',
                    'value': 'true'
                }
            ]
        ),
        Node(
            package='my_robot_description',
            node_executable='spawn_robot1',
            output='screen'
        ),
        Node(
            package='my_robot_description',
            node_executable='spawn_robot2',
            output='screen'
        )
    ])