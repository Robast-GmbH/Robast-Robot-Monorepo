from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

        start_door_handle_detection_node = Node(
                package="door_handle_detector_sim",
                executable="door_handle_detector",
                output="screen"
        )

        ld = LaunchDescription()

        ld.add_action(start_door_handle_detection_node)

        return ld