import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    start_drawer_bridge_cmd = launch_ros.actions.Node(
        package="drawer_bridge",
        executable="drawer_bridge",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        arguments=["--ros-args", "--log-level", "info"],
    )

    ld = LaunchDescription()

    ld.add_action(start_drawer_bridge_cmd)

    return ld
