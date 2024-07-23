from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Parameters

    # Nodes launching commands
    start_nfc_bridge_cmd = launch_ros.actions.Node(
        package='nfc_bridge',
        executable='nfc_bridge',
        output='screen',
        name='reader_1',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    )

    ld = LaunchDescription()

    ld.add_action(start_nfc_bridge_cmd)
    return ld
