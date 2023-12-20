import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():

    # Nodes launching commands
    start_ultrasonic_hc_sr04_cmd = launch_ros.actions.Node(
            package='ultrasonic_hc_sr04',
            executable='ultrasonic_hc_sr04',
            output='screen',
            parameters=[
                {'pigpio_host': 'localhost'},
                {'pigpio_port': '8888'},
                {'trigger_pin': 14},
                {'echo_pin': 18},
                {'topic': 'distance'},
                {'frequency': 10}
            ]
            )

    ld = LaunchDescription()

    ld.add_action(start_ultrasonic_hc_sr04_cmd)

    return ld