from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Parameters
    use_sim_time = True
    autostart = True

    # Nodes launching commands
    start_map_saver_server_cmd = launch_ros.actions.Node(
            package='robast_drawer_gate',
            executable='drawer_gate',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

    ld = LaunchDescription()

    ld.add_action(start_map_saver_server_cmd)

    return ld