from sub_bridges.base_bridge import BaseBridge
from thread_safe_dict import ThreadSafeDict
from roslibpy import Ros, Message
import numpy as np


class NavBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)

        self.start_subscriber(
            "/navigation_remaining_time",
            "builtin_interfaces/msg/Duration",
        )
        self.start_subscriber(
            "/is_navigating",
            "std_msgs/msg/Bool",
        )

        self._goal_pose_publisher = self.start_publisher(
            "/set_goal_pose",
            "geometry_msgs/msg/Pose",
        )
        self._cancel_goal_publisher = self.start_publisher(
            "/cancel_goal",
            "std_msgs/msg/Bool",
        )

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def navigate_to_goal_pose(self, robot_name, goal_pose):
        self.context.update("/is_navigating", {"data": True})
        goal_rotation = self.get_quaternion_from_euler(0, 0, goal_pose[2])

        goal_msg = Message(
            {
                "position": {
                    "x": goal_pose[0],
                    "y": goal_pose[1],
                    "z": 0.0,
                },
                "orientation": {
                    "x": goal_rotation[0],
                    "y": goal_rotation[1],
                    "z": goal_rotation[2],
                    "w": goal_rotation[3],
                },
            }
        )
        self._goal_pose_publisher.publish(goal_msg)
        return True

    def cancel_navigate_to_goal_pose(self):
        self._cancel_goal_publisher.publish(Message({"data": True}))
        return True

    def is_navigating(self):
        is_navigating = self.context.get("/is_navigating")
        if is_navigating is None:
            return False
        else:
            return is_navigating["data"]

    def get_remaining_nav_time(self):
        remaining_time = self.context.get("/navigation_remaining_time")
        if remaining_time is None:
            return 0
        else:
            return remaining_time["sec"]
