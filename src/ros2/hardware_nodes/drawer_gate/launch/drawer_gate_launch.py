from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Parameters
    use_sim_time = True
    autostart = True

    # Action call for testing: ros2 action send_goal /control_drawer communication_interfaces/action/DrawerUserAccess "{drawer_id: 0}"
    # Nodes launching commands
    start_drawer_gate_cmd = launch_ros.actions.Node(
            package='drawer_gate',
            executable='drawer_gate',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

    ld = LaunchDescription()

    ld.add_action(start_drawer_gate_cmd)

    return ld