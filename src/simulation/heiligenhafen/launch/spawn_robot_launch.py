from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    init_x = LaunchConfiguration('init_x', default="8.59")
    init_y = LaunchConfiguration('init_y', default="-13.45")
    init_yaw = LaunchConfiguration('init_yaw', default="3.14")
    robot_name = LaunchConfiguration('robot_name', default="rb_theron")

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='rb_theron',
        description='name of the robot in the simulation')

    spawn_robo_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", robot_name, "-x", init_x, "-y", init_y, "-Y", init_yaw])

    ld = LaunchDescription()
    # arguments
    ld.add_action(declare_robot_name_cmd)

    # nodes
    ld.add_action(spawn_robo_cmd)

    return ld
