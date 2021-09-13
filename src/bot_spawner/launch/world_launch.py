import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node





#export GAZEBO_MODEL_PATH=/workspaces/foxy_ws/src/map_server/models:${GAZEBO_MODEL_PATH}
#export WORLD_MODEL=5OG
#export TURTLEBOT3_MODEL=waffle
#source /usr/share/gazebo/setup.sh

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  
    world = os.path.join(get_package_share_directory('bot_spawner'),
                         'worlds','5OG.world' )
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

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

    ])
