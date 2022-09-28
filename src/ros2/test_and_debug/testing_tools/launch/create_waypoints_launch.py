from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    num_of_waypoints = LaunchConfiguration('num_of_waypoints')
    num_of_waypoints = LaunchConfiguration('random_seed')
    num_of_waypoints = LaunchConfiguration('num_of_waypoints')

    return LaunchDescription([

        DeclareLaunchArgument(
            'num_of_waypoints', default_value='9',
            description='Number of random created waypoints'),

        DeclareLaunchArgument(
            'random_seed', default_value='2',
            description='Random seed for the waypoint creation'),

        DeclareLaunchArgument(
            'random_deviation_goal_pose', default_value='1.5',
            description='Random deviation for goal pose'),

        Node(
            package='testing_tools',
            executable='create_waypoints',
            name='create_waypoints',
            output='screen',
            parameters=[{'num_of_waypoints': num_of_waypoints}]),
    ])