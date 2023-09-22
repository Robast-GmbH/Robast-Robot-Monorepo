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

        #  ROS -> GAZEBO
    robot_trajectory_executor_node = Node(
        package="gazebo_controller_manager",
        executable="robot_trajectory_executor",
        name="rb_theron_robot_trajectory_executor",
        parameters=[gazebo_controller_manager_params],
        output="screen",
    )

    task_solution_executor = Node(
        package="gazebo_controller_manager",
        executable="task_solution_executor",
        name="rb_theron_task_solution_executor",
        output="screen",
    )

    ld.add_action(declare_gazebo_controller_manager_params_cmd)
    ld.add_action(robot_trajectory_executor_node)
    ld.add_action(task_solution_executor)
    return ld
