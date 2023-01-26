from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    # Nodes launching commands
    start_nfc_gate_cmd = launch_ros.actions.Node(
            package='nfc_gate',
            executable='nfc_gate',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

    ld = LaunchDescription()

    ld.add_action(start_nfc_gate_cmd)

    return ld
