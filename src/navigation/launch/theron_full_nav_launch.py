import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLD_MODEL = os.environ['WORLD_MODEL']
ROBOT_MODEL = os.environ['ROBOT_MODEL']
POSE_INIT_X = os.environ['POSE_INIT_X']
POSE_INIT_Y = os.environ['POSE_INIT_Y']
POSE_INIT_Z = os.environ['POSE_INIT_Z']

# export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/map_server/models:${GAZEBO_MODEL_PATH}
# export GAZEBO_PLUGIN_PATH=workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}
# export WORLD_MODEL=5OG
# export ROBOT_MODEL=rb_theron

def generate_launch_description():
    
    navigation_launch_file_dir = os.path.join(get_package_share_directory('navigation'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navigation_launch_file_dir, '/slam_toolbox_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navigation_launch_file_dir, '/robot_localization_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navigation_launch_file_dir, '/nav_without_localization_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
