import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():

    # Nodes launching commands
    start_dryve_d1_bridge_cmd = launch_ros.actions.Node(
            package='dryve_d1_bridge',
            executable='dryve_d1_bridge',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

    ld = LaunchDescription()

    ld.add_action(start_dryve_d1_bridge_cmd)

    return ld
