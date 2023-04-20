# Return a list of nodes we commonly launch for the demo. Nice for testing use.
import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_robot_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit2_door_opening_mechanism_config"),
            "config",
            "rb_theron.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    return robot_description


def get_robot_description_semantic():
    robot_description_semantic_config = load_file(
        "moveit2_door_opening_mechanism_config", "config/rb_theron.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    return robot_description_semantic


def generate_common_hybrid_launch_description():

    moveit_config = MoveItConfigsBuilder("rb_theron", package_name="moveit2_door_opening_mechanism_config").to_moveit_configs()
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = get_robot_description()

    robot_description_semantic = get_robot_description_semantic()

    kinematics_yaml = load_yaml(
        "moveit2_door_opening_mechanism_config", "config/kinematics.yaml"
    )

    # The global planner uses the typical OMPL parameters
    planning_pipelines_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
        "chomp": {
            "planning_plugin": "chomp_interface/CHOMPPlanner",
        }
    }
    ompl_planning_yaml = load_yaml(
        "moveit2_door_opening_mechanism_config", "config/ompl_planning.yaml"
    )
    planning_pipelines_config["ompl"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "moveit2_door_opening_mechanism_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Any parameters that are unique to your plugins go here
    common_hybrid_planning_param = load_yaml(
        "moveit2_door_opening_mechanism_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )
    global_planner_param = load_yaml(
        "moveit2_door_opening_mechanism_hybrid_planning", "config/global_planner.yaml"
    )
    local_planner_param = load_yaml(
        "moveit2_door_opening_mechanism_hybrid_planning", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "moveit2_door_opening_mechanism_hybrid_planning", "config/hybrid_planning_manager.yaml"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="whether to use sim time or not",
    )

    # Generate launch description with multiple components
    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    common_hybrid_planning_param,
                    global_planner_param,
                    # robot_description,
                    # robot_description_semantic,
                    # kinematics_yaml,
                    # planning_pipelines_config,
                    # moveit_controllers,
                    moveit_config.to_dict(),
                    # {"epsilon": 0.000010},
                    # {"orientation_vs_position": 0.0}
                    # {"use_sim_time": use_sim_time}
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    common_hybrid_planning_param,
                    local_planner_param,
                    moveit_config.to_dict(),                    
                    # {"use_sim_time": use_sim_time}
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                    moveit_config.to_dict(),
                    # {"use_sim_time": use_sim_time}
                ],
            ),
        ],
        output="screen",
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit2_door_opening_mechanism_hybrid_planning")
        + "/config/hybrid_planning_demo.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    {"use_sim_time": use_sim_time},
                    ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit2_door_opening_mechanism_hybrid_planning"),
        "config",
        "demo_controller.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    panda_joint_group_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_joint_group_position_controller",
            "-c",
            "/controller_manager",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(container)
    ld.add_action(rviz_node)

    # launched_nodes = [
    #     container,
    #     rviz_node,
    #     # ros2_control_node,
    #     # panda_joint_group_position_controller_spawner,
    #     declare_use_sim_time_cmd,
    # ]

    return ld
