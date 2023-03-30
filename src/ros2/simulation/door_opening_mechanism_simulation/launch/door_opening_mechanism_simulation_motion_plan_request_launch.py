from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("rb_theron", package_name="moveit2_door_opening_mechanism_config").to_moveit_configs()

    ld = LaunchDescription()

    door_opening_mechanism_simulation_node = Node(
        package="door_opening_mechanism_simulation",
        executable="door_opening_mechanism_simulation_motion_plan_request",
        name="door_opening_mechanism_simulation",
        parameters=[
            {"planning_plugin": "ompl_interface/OMPLPlanner"},
            {"use_sim_time": True},
            moveit_config.to_dict()
        ],
        output="screen",
    )

    ld.add_action(door_opening_mechanism_simulation_node)
    return ld
