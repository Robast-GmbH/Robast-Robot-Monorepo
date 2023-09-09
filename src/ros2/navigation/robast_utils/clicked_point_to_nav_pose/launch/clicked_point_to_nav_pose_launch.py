from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    start_bt_server_collection_cmd = Node(
        package='clicked_point_to_nav_pose',
        executable='clicked_point_to_nav_pose',
        name='clicked_point_to_nav_pose',
        output='screen')

    ld = LaunchDescription()
    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(start_bt_server_collection_cmd)

    return ld
