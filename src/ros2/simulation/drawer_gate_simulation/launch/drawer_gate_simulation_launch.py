
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    drawer_gate_simulation_node = Node(
                package='drawer_gate_simulation',
                executable='drawer_gate_simulation',
                name="drawer_gate_simulation",
                parameters=[
                            {"time_until_drawer_closes_automatically_in_ms": 5000},
                            {"moveit2_planning_group_name": "drawer_planning_group"},
                           ],
                output='screen')

    ld.add_action(drawer_gate_simulation_node)
    return ld
