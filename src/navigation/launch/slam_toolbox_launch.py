import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_MODEL = os.environ['WORLD_MODEL']
POSE_INIT_X = os.environ['POSE_INIT_X']
POSE_INIT_Y = os.environ['POSE_INIT_Y']
POSE_INIT_Z = os.environ['POSE_INIT_Z']

def generate_launch_description():

    slam_toolbox_params_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'slam_toolbox_params_offline.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('navigation'), 'config', 'nav2_default_view.rviz')
    map_file_posegraph = os.path.join(get_package_share_directory('navigation'), 'maps', WORLD_MODEL)

    print('map_file_posegraph: {}'.format(map_file_posegraph))   

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        Node(            
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_toolbox_params_yaml,
                {'map_file_name': map_file_posegraph},
                {'map_start_pose': [float(POSE_INIT_X), float(POSE_INIT_Y), 3.14]},
                {'mode': 'mapping'},
                {'use_sim_time': use_sim_time}
            ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['slam_toolbox']}]),

    ])
