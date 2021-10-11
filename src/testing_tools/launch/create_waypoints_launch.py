from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    num_of_waypoints = LaunchConfiguration('num_of_waypoints')

    return LaunchDescription([

        DeclareLaunchArgument(
            'num_of_waypoints', default_value='9',
            description='Number of random created waypoints'),

        Node(
            package='testing_tools',
            executable='create_waypoints',
            name='create_waypoints',
            output='screen',
            parameters=[{'num_of_waypoints': num_of_waypoints}]),
    ])