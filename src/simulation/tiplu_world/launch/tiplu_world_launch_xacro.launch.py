import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


WORLD_MODEL = os.environ['WORLD_MODEL']
ROBOT_MODEL = os.environ['ROBOT_MODEL']
POSE_INIT_X = os.environ['POSE_INIT_X']
POSE_INIT_Y = os.environ['POSE_INIT_Y']
POSE_INIT_Z = os.environ['POSE_INIT_Z']

# export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/tiplu_world/models:${GAZEBO_MODEL_PATH}
# export GAZEBO_PLUGIN_PATH=workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}
# export WORLD_MODEL=5OG
# export ROBOT_MODEL=rb_theron


def get_robot_xml():
    xacro_file = os.path.join(get_package_share_directory('rb_theron_description'), 'robots', 'rb_theron.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_xml = robot_description_config.toxml()
    return robot_xml


def get_robot_spawn_args():
    initial_pose = '{position: ' +\
        '{x: ' + POSE_INIT_X + ', ' +\
        'y: ' + POSE_INIT_Y + ', ' +\
        'z: ' + POSE_INIT_Z + '}' + ', ' +\
                   'orientation: ' +\
        '{z: 1, w: 0}' + '}'

    spawn_args = '{name: \"' + ROBOT_MODEL + '\",' +\
                 'xml: \"' + get_robot_xml().replace('"', '\\"') + '\",' +\
                 'reference_frame: \"world\"' + ',' +\
                 'initial_pose: ' + initial_pose + '}'
    return spawn_args


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(get_package_share_directory('tiplu_world'),
                         'worlds',
                         WORLD_MODEL,
                         WORLD_MODEL + '.model')

    launch_file_dir = os.path.join(get_package_share_directory('tiplu_world'), 'launch')
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

        # Spawn Robot in World
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', get_robot_spawn_args()],
            output='screen'),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": get_robot_xml()},
                {'use_sim_time': use_sim_time}],
            output="screen"),
    ])
