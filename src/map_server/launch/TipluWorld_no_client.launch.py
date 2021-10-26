import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
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

def get_robot_spawn_args():
    robot_sdf = os.path.join(get_package_share_directory('map_server'),
                         'models',
                         ROBOT_MODEL,
                         ROBOT_MODEL + '.sdf')
    robot_xml = open(robot_sdf, 'r').read()
    robot_xml = robot_xml.replace('"', '\\"')

    initial_pose = '{position: ' +\
                    '{x: ' + POSE_INIT_X + ', ' +\
                     'y: ' + POSE_INIT_Y + ', ' +\
                     'z: ' + POSE_INIT_Z + '}' + ', ' +\
                   'orientation: ' +\
                    '{z: 1, w: 0}' + '}'

    spawn_args = '{name: \"' + ROBOT_MODEL + '\",' +\
                 'xml: \"' + robot_xml + '\",' +\
                 'initial_pose: ' + initial_pose + '}'
    return spawn_args

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

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        #     ),
        # ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        # Spawn Robot in World
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', get_robot_spawn_args()],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
