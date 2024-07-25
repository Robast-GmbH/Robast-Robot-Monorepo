import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)

"""
    Launches a demo to use together with gz sim

    Includes
     * static_virtual_joint_tfs
     * move_group
     * moveit_rviz
"""


def generate_launch_description():
    launch_arguments = {
        "ros2_control_hardware_type": "gz_ros2_control",
        "position_joint_type": "prismatic",
    }

    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "humble":
        planning_pipelines = ["ompl_humble"]
    elif ros_distro == "iron":
        planning_pipelines = ["ompl_iron"]
    else:
        raise Exception("Unknown ROS distro: " + ros_distro)

    moveit_config = (
        MoveItConfigsBuilder(
            "rb_theron",
            package_name="moveit_door_opening_mechanism_config",
        )
        .robot_description(
            file_path="config/rb_theron.urdf.xacro", mappings=launch_arguments
        )
        .robot_description_semantic(file_path="config/rb_theron.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .sensors_3d(file_path="config/sensors_3d_simulation.yaml")
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
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
    ld.add_action(generate_static_virtual_joint_tfs_launch(moveit_config))

    # Load ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                move_group_capabilities,
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_door_opening_mechanism_config")
        + "/config/moveit_gazebo.rviz"
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
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    return ld
