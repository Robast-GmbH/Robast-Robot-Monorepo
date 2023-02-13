from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    door_opening_mechanism_simulation_node = Node(
        package="door_opening_mechanism_simulation",
        executable="door_opening_mechanism_simulation",
        name="door_opening_mechanism_simulation",
        parameters=[
            {"time_until_drawer_closes_automatically_in_ms": 5000},
            {"moveit2_planning_group_name": "door_opening_mechanism"},
            {"use_sim_time": True},
            {"planning_plugin": "ompl_interface/OMPLPlanner"},
        ],
        output="screen",
    )

    ld.add_action(door_opening_mechanism_simulation_node)
    return ld
