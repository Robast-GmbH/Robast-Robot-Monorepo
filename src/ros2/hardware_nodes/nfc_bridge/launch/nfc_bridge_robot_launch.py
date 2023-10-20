from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Parameters

    # Nodes launching commands
    start_nfc_bridge_cmd1 = launch_ros.actions.Node(
        package='nfc_bridge',
        executable='nfc_bridge',
        output='screen',
        name='reader_l',
        parameters=[{'serial_port_path', '/dev/robast/robast_nfc_1'}],
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    )

    start_nfc_bridge_cmd2 = launch_ros.actions.Node(
        package='nfc_bridge',
        executable='nfc_bridge',
        output='screen',
        name='reader_r',
        parameters=[{'serial_port_path', '/dev/robast/robast_nfc_2'}],
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    )
    ld = LaunchDescription()
    ld.add_action(start_nfc_bridge_cmd1)
    ld.add_action(start_nfc_bridge_cmd2)
    return ld
