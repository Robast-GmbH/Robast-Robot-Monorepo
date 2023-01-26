
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    drawer_simulation_node = Node(
                package='drawer_simulation', 
                executable='drawer_simulation',
                name="drawer_simulation",
                parameters=[
                            {"time_until_drawer_closes_automatically_in_ms": 5000},
                           ],
                output='screen')

    ld.add_action(drawer_simulation_node)          
    return ld
