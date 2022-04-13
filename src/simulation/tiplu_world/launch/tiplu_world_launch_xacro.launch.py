import os
from sys import prefix
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


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
    robot_description_config = xacro.process_file(xacro_file, mappings={'prefix': 'robot_'})
    robot_xml = robot_description_config.toxml()
    return robot_xml


def get_robot_spawn_args(initial_pose, robot_model, robot_xml):

    spawn_args = '{name: \"' + robot_model + '\",' +\
                 'xml: \"' + robot_xml.replace('"', '\\"') + '\",' +\
                 'reference_frame: \"world\"' + ',' +\
                 'initial_pose: ' + initial_pose + '}'
    return spawn_args


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_pose = LaunchConfiguration('initial_pose')
    world_model = LaunchConfiguration('world_model')
    robot_model = LaunchConfiguration('robot_model')
    frame_prefix = LaunchConfiguration('frame_prefix')
    xacro_path = LaunchConfiguration('xacro_path')
    robot_xml = xacro.process_file(os.path.join(get_package_share_directory(
        'rb_theron_description'), 'robots', 'rb_theron.urdf.xacro'), mappings={'prefix': 'robot_'}).toxml()

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_initial_pose_cmd = DeclareLaunchArgument(
        'initial_pose',
        default_value='{position: ' +
        '{x: ' + POSE_INIT_X + ', ' +
        'y: ' + POSE_INIT_Y + ', ' +
        'z: ' + POSE_INIT_Z + '}' + ', ' +
        'orientation: ' +
        '{z: 1, w: 0}' + '}',
        description='Top-level namespace')

    declare_world_model_cmd = DeclareLaunchArgument(
        'world_model',
        default_value=os.path.join(get_package_share_directory('tiplu_world'),
                                   'worlds',
                                   '5OG',
                                   '5OG' + '.model'),
        description='math to the world model')

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='rb_theron',
        description='name of the robot-model')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='whether to use sim time or not')

    declare_prefix_cmd = DeclareLaunchArgument(
        'frame_prefix',
        default_value='robot_',
        description='frame prefix'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'xacro_path',
        default_value=os.path.join(get_package_share_directory(
            'rb_theron_description'), 'robots', 'rb_theron.urdf.xacro'),
        description='path to the xacro file of the robot')

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[
                {'use_sim_time': use_sim_time,
                 'robot_description': robot_xml,
                 }],
        output="screen"),

    exec_use_sim_time_cmd = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        output='screen'),

    exec_spawn_robot_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity',
             get_robot_spawn_args(initial_pose=initial_pose, robot_model=robot_model, robot_xml=robot_xml)],
        output='screen')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    start_gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_model}.items(),
    ),

    start_gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    ),

    print('world_file_path : {}'.format(world_model))

    ld = LaunchDescription()

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_initial_pose_cmd)
    ld.add_action(declare_world_model_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_prefix_cmd)

    # included launches
    ld.add_action(start_gzserver_cmd)
    ld.add_action(start_gzclient_cmd)

    # execute
    ld.add_action(exec_spawn_robot_cmd)
    ld.add_action(exec_use_sim_time_cmd)

    # nodes
    ld.add_action(start_robot_state_publisher_cmd)

    return ld

    # launch_file_dir = os.path.join(get_package_share_directory('tiplu_world'), 'launch')
    # rb_theron_controller_dir = os.path.join(get_package_share_directory(
    #     'rb_theron_description'), 'config', 'rb_theron_controller.yaml')

    # Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {"robot_description": get_robot_xml()},
    #         {'use_sim_time': use_sim_time},
    #         rb_theron_controller_dir],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     },
    # ),

    # Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diffbot_base_controller"],
    #     output="screen",
    # ),

    # Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_state_broadcaster"],
    #     output="screen",
    # ),
