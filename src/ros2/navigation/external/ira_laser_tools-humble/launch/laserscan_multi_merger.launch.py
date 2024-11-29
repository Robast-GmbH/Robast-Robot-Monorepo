from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    laserscan_topics_arg = DeclareLaunchArgument(
        'laserscan_topics',
        default_value='/robot/front_laser/scan /robot/rear_laser/scan',
        description='List of laser scan topics to merge separated by space'
    )

    laserscan_topics = LaunchConfiguration('laserscan_topics')

    return LaunchDescription([
        laserscan_topics_arg,
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            output='screen',
            parameters=[
                {'destination_frame': 'robot/base_link'},
                {'cloud_destination_topic': '/merged_cloud'},
                {'scan_destination_topic': '/robot/merged_laser/scan'},
                {'laserscan_topics': laserscan_topics},
                {'angle_min': -3.14},
                {'angle_max': 3.14},
                {'angle_increment': 0.003},
                {'scan_time': 0.0},
                {'range_min': 0.1},
                {'range_max': 40.0},
            ]
        )
    ])