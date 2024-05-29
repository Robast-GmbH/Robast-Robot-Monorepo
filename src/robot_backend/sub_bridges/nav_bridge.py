from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros, Message
import numpy as np


LOOKUP_TABLE_RESOLUTION = 2


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

        self._sin_lookup = [
            np.sin(np.radians(x / 2)) for x in range(0, 360, LOOKUP_TABLE_RESOLUTION)
        ]
        self._cos_lookup = [
            np.cos(np.radians(x / 2)) for x in range(0, 360, LOOKUP_TABLE_RESOLUTION)
        ]

        # Navigation is blocked by canceling the current goal and pretending the robot is still navigating
        self.is_nav_blocked = False
        self.goal_pose = None

    def get_quaternion_from_euler(self, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = 0
        qy = 0
        yaw_degrees = int(np.degrees(yaw)) % 360
        index = yaw_degrees // LOOKUP_TABLE_RESOLUTION
        qz = self._sin_lookup[index]
        qw = self._cos_lookup[index]

        return {"x": qx, "y": qy, "z": qz, "w": qw}

    def navigate_to_goal_pose(self, goal_pose):
        self.goal_pose = goal_pose
        if not self.is_nav_blocked:
            self.context["/is_navigating"] = {"data": True}
            goal_msg = Message(
                {
                    "position": {
                        "x": goal_pose[0],
                        "y": goal_pose[1],
                        "z": 0.0,
                    },
                    "orientation": self.get_quaternion_from_euler(goal_pose[2]),
                }
            )
            self._goal_pose_publisher.publish(goal_msg)
        return True

    def cancel_navigate_to_goal_pose(self):
        self._cancel_goal_publisher.publish(Message({"data": True}))
        return True

    def is_navigating(self):
        # Return True if nav_is_blocked so rmf won't evalauate the current rmf nav goal as reached
        if self.is_nav_blocked:
            return True
        try:
            return self.context["/is_navigating"]["data"]
        except KeyError:
            return False

    def get_remaining_nav_time(self):
        try:
            return self.context["/navigation_remaining_time"]["sec"]
        except KeyError:
            return 0

    def block_nav(self):
        self.is_nav_blocked = True
        self.cancel_navigate_to_goal_pose()
        return True

    def unblock_nav(self):
        self.is_nav_blocked = False
        if self.goal_pose is not None:
            self.navigate_to_goal_pose(self.goal_pose)
        return True
