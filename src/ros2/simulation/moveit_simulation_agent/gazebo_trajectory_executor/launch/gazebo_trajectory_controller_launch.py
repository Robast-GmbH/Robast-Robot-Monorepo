import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    gazebo_trajectory_executor_params = LaunchConfiguration('gazebo_trajectory_executor_params')

    declare_gazebo_trajectory_executor_params_cmd = DeclareLaunchArgument(
        'gazebo_trajectory_executor_params',
        default_value=os.path.join(get_package_share_directory('gazebo_trajectory_executor'), 'config', 'gazebo_trajectory_executor_rb_theron.yaml'),
        description='path to the config yaml file')


    #  ROS -> GAZEBO,  joint trajectory controller
    joint_trajectory_executor = Node(
        package="gazebo_trajectory_executor",
        executable="joint_trajectory_executor",
        name="rb_theron_joint_trajectory_executor",
        parameters=[gazebo_trajectory_executor_params],
        output="screen",
    )

    ld.add_action(declare_gazebo_trajectory_executor_params_cmd)
    ld.add_action(joint_trajectory_executor)
    return ld
