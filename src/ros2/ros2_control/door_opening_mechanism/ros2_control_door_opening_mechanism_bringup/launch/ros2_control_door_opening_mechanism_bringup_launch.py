import os
import xacro
import yaml
import atexit

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def shutdown_dryve_d1():
    print("Shutting down dryve d1 motor controller now!")
    os.system('ros2 run dryve_d1_bridge shutdown_dryve_d1')


def generate_launch_description():

    # Get URDF via xacro
    robot_xml = xacro.process_file(
        os.path.join(
            get_package_share_directory("rb_theron_description"),
            "robots",
            os.environ['robot'] + ".urdf.xacro",
        ),
        mappings={"prefix": os.environ['prefix']},
    ).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="whether to use sim time or not",
    )

    robot_description = {"robot_description": robot_xml}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_door_opening_mechanism_bringup"),
            "config",
            "door_mechanism_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time}],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )


    robot_state_publisher_spawner = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_xml}],
        output="screen",
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_state_publisher_spawner)
    # ld.add_action(forward_velocity_controller_spawner)
    ld.add_action(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)

    # This would be probably much nice to use RegisterEventHandler to trigger the shutdown, but unfortunately we
    # did not get this running. Therefore we shutdown the dryve d1 with the "atexit" library.
    atexit.register(shutdown_dryve_d1)

    return ld
