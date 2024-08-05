from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="disinfection_module",
                executable="disinfection_publisher",
                name="disinfection_publisher",
                parameters=[
                    {"read_in_disinfection_switch_counter_limit": 10},
                    {"disinfection_switch_threshold": 0.8},
                    {"disinfection_timer_period_in_sec": 0.02},
                ],
            ),
        ]
    )
