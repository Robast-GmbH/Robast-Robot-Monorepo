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


# export GAZEBO_MODEL_PATH=/workspaces/Robast_RosTheron/src/tiplu_world/models:${}
# export GAZEBO_PLUGIN_PATH=workspaces/Robast_RosTheron/gaz/plugins:${GAZEBO_PLUGIN_PATH}
# export WORLD_MODEL=5OG
# export ROBOT_MODEL=rb_theron


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_pose_x = LaunchConfiguration('initial_pose_x', default="8.59")
    initial_pose_y = LaunchConfiguration('initial_pose_y', default="-13.45")
    initial_pose_Yaw = LaunchConfiguration('initial_pose_Yaw', default="3.14")
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

    declare_world_model_cmd = DeclareLaunchArgument(
        'world_model',
        default_value=os.path.join(get_package_share_directory('tiplu_world'), 'worlds', '5OG', '5OG' + '.model'),
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

    declare_xacro_path_cmd = DeclareLaunchArgument(
        'xacro_path',
        default_value=os.path.join(get_package_share_directory(
            'rb_theron_description'), 'robots', 'rb_theron.urdf.xacro'),
        description='path to the xacro file of the robot')

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                {'use_sim_time': use_sim_time,
                 'robot_description': robot_xml,
                 }],
        output="screen")

    spawn_robo_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", robot_model, "-x", initial_pose_x, "-y", initial_pose_y, "-Y", initial_pose_Yaw])

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    start_gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_model}.items(),
    )

    start_gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    print('world_file_path : {}'.format(world_model))

    ld = LaunchDescription()

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_world_model_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_xacro_path_cmd)

    # included launches
    ld.add_action(start_gzserver_cmd)
    ld.add_action(start_gzclient_cmd)

    # nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robo_cmd)

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
