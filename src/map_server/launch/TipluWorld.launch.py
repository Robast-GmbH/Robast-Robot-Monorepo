import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLD_MODEL = os.environ['WORLD_MODEL']

# export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/map_server/models:${GAZEBO_MODEL_PATH}
# export GAZEBO_PLUGIN_PATH=workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}
# export WORLD_MODEL=5OG
# export ROBOT_MODEL=rb_theron

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(get_package_share_directory('map_server'),
                         'worlds',
                         WORLD_MODEL,
                         WORLD_MODEL + '.model')
    launch_file_dir = os.path.join(get_package_share_directory('map_server'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    print('world_file_path : {}'.format(world))   

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
