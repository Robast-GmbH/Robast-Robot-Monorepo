import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('map_server'), 'config', 'nav2.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'config', 'nav2_default_view.rviz')
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', '5OG/waffle.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[nav2_yaml, {'yaml_filename':map_file} ]),
            
        Node(
            package='nav2_amcl',
            node_executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[nav2_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen')
        ])