import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    is_simulation = os.environ["is_simulation"]

    if is_simulation == 'True':
        use_sim_time = True
    feuerplan_publisher_dir = get_package_share_directory('feuerplan_publisher')

    default_feuerplan_image_path = os.path.join(feuerplan_publisher_dir, 'images','slide5.jpg')

    feuerplan_path = LaunchConfiguration('feuerplan_path',  default = default_feuerplan_image_path)
    confidence_threshold = LaunchConfiguration('confidence_threshold',  default = 0.7)

    declare_feuerplan_image_cmd = DeclareLaunchArgument(
        'feuerplan_path',
        default_value=feuerplan_path,
        description='Path to the feuerplan image')
    
    declare_confidence_threshold_cmd = DeclareLaunchArgument(
        'confidence_threshold',
        default_value=confidence_threshold,
        description='Confidence threshold for matches')
    
    start_feuerplan_publisher = Node(
        package='feuerplan_publisher',
        executable='feuerplan_publisher_node',
        parameters=[
                    {'feuerplan_path': feuerplan_path},
                    {'confidence_threshold': confidence_threshold},
                    {'use_sim_time': use_sim_time}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_feuerplan_image_cmd)
    ld.add_action(declare_confidence_threshold_cmd)
    ld.add_action(start_feuerplan_publisher)

    return ld