from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
        # Parameters
        use_sim_time = True
        autostart = True

    # Nodes launching commands
        
        start_drawer_gate_cmd = launch_ros.actions.Node(
            package='robast_drawer_gate',
            executable='drawer_gate',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

        start_nfc_gate_cmd = launch_ros.actions.Node(
            package='robast_nfc_gate',
            executable='nfc_gate',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )


        start_drawer_manager_cmd = launch_ros.actions.Node(
            package='robast_drawer_manager',
            executable='drawer_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

        start_drawer_sym_cmd = launch_ros.actions.Node(
            package='robast_drawer_sym',
            executable='drawer_sym',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )
        ld = LaunchDescription()

        ld.add_action(start_drawer_gate_cmd)
        ld.add_action(start_nfc_gate_cmd)
        ld.add_action(start_drawer_manager_cmd)
        ld.add_action(start_drawer_sym_cmd)
        return ld