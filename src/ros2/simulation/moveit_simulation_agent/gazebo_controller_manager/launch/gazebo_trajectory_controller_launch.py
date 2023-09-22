import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    gazebo_controller_manager_params = LaunchConfiguration('gazebo_controller_manager_params')

    declare_gazebo_controller_manager_params_cmd = DeclareLaunchArgument(
        'gazebo_controller_manager_params',
        default_value=os.path.join(get_package_share_directory('gazebo_controller_manager'), 'config', 'gazebo_controller_rb_theron.yaml'),
        description='path to the config yaml file')


    #  ROS -> GAZEBO,  joint trajectory controller
    joint_trajectory_controller = Node(
        package="gazebo_controller_manager",
        executable="joint_trajectory_controller",
        name="rb_theron_joint_trajectory_controller",
        parameters=[gazebo_controller_manager_params],
        output="screen",
    )

    ld.add_action(declare_gazebo_controller_manager_params_cmd)
    ld.add_action(joint_trajectory_controller)
    return ld
