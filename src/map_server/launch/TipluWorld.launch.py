import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
WORLD_MODEL = os.environ['WORLD_MODEL']

#export GAZEBO_MODEL_PATH=/workspaces/foxy_ws/src/map_server/models:${GAZEBO_MODEL_PATH}
#export WORLD_MODEL=5OG
#export TURTLEBOT3_MODEL=waffle
#source /usr/share/gazebo/setup.sh

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = WORLD_MODEL + '/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('map_server'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('map_server'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),       
    ])