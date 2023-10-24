
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    drawer_bridge_simulation_node = Node(
                package='drawer_bridge_simulation',
                executable='drawer_bridge_simulation',
                name="drawer_bridge_simulation",
                parameters=[
                            {"time_until_drawer_closes_automatically_in_ms": 5000},
                            {"moveit2_planning_group_name": "drawer_planning_group"},
                            {"num_of_drawers": 5},
                           ],
                output='screen')

    ld.add_action(drawer_bridge_simulation_node)
    return ld
