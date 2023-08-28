from launch_ros.actions import Node
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
 
    ff_server_api_node = Node(
        package='free_fleet_client_direct',
        executable='client_direct',
    )
 
    return LaunchDescription([
     ff_server_api_node
    ])