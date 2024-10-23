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
            "/goal_status",
            "std_msgs/msg/String",
        )

        self.__goal_pose_publisher = self.start_publisher(
            "/set_goal_pose",
            "geometry_msgs/msg/Pose",
        )
        self.__cancel_goal_publisher = self.start_publisher(
            "/cancel_goal",
            "std_msgs/msg/Bool",
        )
        self.__enable_reorientation_publisher = self.start_publisher(
            "/is_reorientation_enabled",
            "std_msgs/msg/Bool",
        )

        self.__sin_lookup: list[float] = [
            np.sin(np.radians(x / 2)) for x in range(0, 360, LOOKUP_TABLE_RESOLUTION)
        ]
        self.__cos_lookup: list[float] = [
            np.cos(np.radians(x / 2)) for x in range(0, 360, LOOKUP_TABLE_RESOLUTION)
        ]

        # Navigation is blocked by canceling the current goal and pretending the robot is still navigating
        self.__is_nav_blocked = False
        self.__goal_pose = None

    def navigate_to_goal_pose(
        self,
        goal_pose: tuple[float, float, float],
        use_reorientation: bool | None,
    ) -> bool:
        if use_reorientation is not None:
            self.__enable_reorientation_publisher.publish(
                Message({"data": use_reorientation})
            )
        self.__goal_pose = goal_pose
        if not self.__is_nav_blocked:
            self.__clear_goal_status()
            goal_msg = Message(
                {
                    "position": {
                        "x": goal_pose[0],
                        "y": goal_pose[1],
                        "z": 0.0,
                    },
                    "orientation": self.__get_quaternion_from_euler(goal_pose[2]),
                }
            )
            self.__goal_pose_publisher.publish(goal_msg)
        return True

    def cancel_navigate_to_goal_pose(self) -> bool:
        self.__cancel_goal_publisher.publish(Message({"data": True}))
        return True

    def get_remaining_nav_time(self) -> int:
        try:
            return self.context["/navigation_remaining_time"]["sec"]
        except KeyError:
            return 0

    def get_is_nav_blocked(self) -> bool:
        return self.__is_nav_blocked

    def block_nav(self) -> bool:
        self.__is_nav_blocked = True
        self.cancel_navigate_to_goal_pose()
        return True

    def unblock_nav(self) -> bool:
        self.__is_nav_blocked = False
        if self.__goal_pose is not None:
            self.navigate_to_goal_pose(self.__goal_pose, None)
        return True

    def requires_replan(self) -> bool:
        if self.__is_nav_blocked:
            return False
        try:
            if "/goal_status" in self.context and self.context["/goal_status"]:
                return self.context["/goal_status"]["data"] == "ABORTED"
            return False
        except KeyError:
            return False

    def is_navigation_completed(self) -> bool:
        try:
            if "/goal_status" in self.context and self.context["/goal_status"]:
                return self.context["/goal_status"]["data"] == "SUCCEEDED"
            return False
        except KeyError:
            return False

    def __get_quaternion_from_euler(self, yaw: float) -> dict[str, float]:
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
        qz = self.__sin_lookup[index]
        qw = self.__cos_lookup[index]

        return {"x": qx, "y": qy, "z": qz, "w": qw}

    def __clear_goal_status(self) -> None:
        self.context["/goal_status"] = None
