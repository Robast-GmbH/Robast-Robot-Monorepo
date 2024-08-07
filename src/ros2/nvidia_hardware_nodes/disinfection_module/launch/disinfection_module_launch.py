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
                    {"read_in_disinfection_switch_counter_limit": 15},
                    {"disinfection_switch_threshold": 0.35},
                    {"is_pulldown": True},
                    {"disinfection_timer_period_in_sec": 0.01},
                    {"num_of_first_readings_to_ignore": 2},
                ],
            ),
        ]
    )
