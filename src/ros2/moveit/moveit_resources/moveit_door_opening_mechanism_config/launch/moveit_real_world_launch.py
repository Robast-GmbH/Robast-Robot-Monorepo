import os
import atexit
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

"""
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
"""

PREFIX = "robot/"

def generate_launch_description():
    common_launch_arguments = {
        "prefix": PREFIX,
        "ros2_control_hardware_type": "dryve_d1",
        "ros2_control_hardware_type_positon_joint": "real_life",
        "position_joint_type": "prismatic",
        "model_door_opening_mechanism": "true",
        "model_module_cage": "false",
        "model_sensors": "false",
    }

    launch_arguments_ros2_control = {
        common_launch_arguments.items(),
        "robot_description_path": "config/rb_theron.urdf.xacro",
        "robot_description_semantic_file_path": "config/rb_theron_real_world.srdf",
        "trajectory_execution_file_path": "config/moveit_controllers_real_world.yaml",
        "sensor_3d_file_path": "config/sensors_3d_real_world.yaml",
        "joint_limits_file_path": "config/joint_limits_real_world.yaml",
    }

    launch_arguments_robot_state_publisher = {
        common_launch_arguments.items(),
        "use_sim_time": "false",
    }
    launch_arguments_robot_state_publisher["robot_description_path"] = os.path.join(
        get_package_share_directory("rb_theron_description"),
        "robots",
        "rb_theron_arm.urdf.xacro",
    ),

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
            file_path=launch_arguments_ros2_control["robot_description_path"], mappings=common_launch_arguments
        )
        .robot_description_semantic(file_path=launch_arguments_ros2_control["robot_description_semantic_file_path"])
        .trajectory_execution(file_path=launch_arguments_ros2_control["trajectory_execution_file_path"])
        .planning_pipelines(pipelines=planning_pipelines)
        .sensors_3d(file_path=launch_arguments_ros2_control["sensor_3d_file_path"])
        .joint_limits(file_path=launch_arguments_ros2_control["joint_limits_file_path"])
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    static_virtual_joint_cmd = generate_static_virtual_joint_tfs_launch(moveit_config)

    # Load ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    move_group_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # Start ros2 control controller
    launch_ros2_control_cmd =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_config"),
        "launch",
        "ros2_control_real_world_launch.py")),
        launch_arguments=launch_arguments_ros2_control.items(),
    )

    # Start robot state publisher
    launch_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("moveit_door_opening_mechanism_config"),
            "launch",
            "robot_state_publisher_launch.py")),
        launch_arguments=launch_arguments_robot_state_publisher.items(),
    )

    ld.add_action(static_virtual_joint_cmd)
    ld.add_action(move_group_cmd)
    ld.add_action(launch_ros2_control_cmd)
    ld.add_action(launch_robot_state_publisher_cmd)

    return ld
