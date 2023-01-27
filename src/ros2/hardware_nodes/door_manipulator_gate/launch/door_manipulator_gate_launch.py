import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():

    # Nodes launching commands
    start_door_manipulator_gate_cmd = launch_ros.actions.Node(
            package='door_manipulator_gate',
            executable='door_manipulator_gate',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            )

    ld = LaunchDescription()

    ld.add_action(start_door_manipulator_gate_cmd)

    return ld
