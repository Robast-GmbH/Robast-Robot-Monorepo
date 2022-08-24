import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_localization_params_yaml = os.path.join(get_package_share_directory(
        'robast_nav_launch'), 'config', 'robot_localization_ekf_odom_to_base.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_params_yaml,
                    {'use_sim_time': use_sim_time}])

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_robot_localization_cmd)

    return ld
