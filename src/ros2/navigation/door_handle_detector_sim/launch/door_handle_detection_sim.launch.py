from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument

def generate_launch_description():

        start_door_handle_detection_node = Node(
                package="door_handle_detector_sim",
                executable="door_handle_detector",
                output="screen"
        )

        ld = LaunchDescription()

        ld.add_action(start_door_handle_detection_node)

        return ld