import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)

"""
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
"""

def generate_launch_description():

    launch_arguments = {
        "ros2_control_hardware_type": "mock_components",
    }

    moveit_config = (
        MoveItConfigsBuilder("rb_theron", package_name="moveit_door_opening_mechanism_rotating_arm_config")
        .robot_description(file_path="config/rb_theron.urdf.xacro", mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="whether to use sim time or not",
    )
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )


    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    ld.add_action(
        generate_static_virtual_joint_tfs_launch(moveit_config)
    )

    # Load ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    ld.add_action(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    ))

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_door_opening_mechanism_rotating_arm_config") + "/config/moveit.rviz"
    )
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
    )

    # State Publisher
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[
                moveit_config.robot_description,
            ],
        )
    )

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_rotating_arm_config"),
        "config",
        "ros2_controllers_simulation.yaml",
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, ros2_controllers_path, {"use_sim_time": use_sim_time}],
            output="both",
        )
    )
    
    # Load controllers
    for controller in [
        "joint_state_broadcaster",
        "joint_trajectory_controller",
    ]:
        ld.add_action(
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        )

    return ld