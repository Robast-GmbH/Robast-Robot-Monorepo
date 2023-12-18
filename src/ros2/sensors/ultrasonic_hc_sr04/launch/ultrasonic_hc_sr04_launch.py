import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():

    # Nodes launching commands
    start_ultrasonic_hc_sr04_cmd = launch_ros.actions.Node(
            package='ultrasonic_hc_sr04',
            executable='ultrasonic_hc_sr04',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

    ld = LaunchDescription()

    ld.add_action(start_ultrasonic_hc_sr04_cmd)

    return ld