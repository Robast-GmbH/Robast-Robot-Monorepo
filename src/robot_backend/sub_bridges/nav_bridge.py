from sub_bridges.base_bridge import BaseBridge
from thread_safe_dict import ThreadSafeDict
from roslibpy import Ros, Message


class NavBridge(BaseBridge):
    def __init__(self, ros: Ros, context: ThreadSafeDict) -> None:
        super().__init__(ros, context)

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

    def navigate_to_goal_pose(self, robot_name, goal_pose):
        self.context.update("/is_navigating", {"data": True})
        goal_msg = Message(
            {
                "position": {
                    "x": goal_pose[0],
                    "y": goal_pose[1],
                    "z": 0.0,
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0,
                },
            }
        )
        self._goal_pose_publisher.publish(goal_msg)
        return True

    def cancel_navigate_to_goal_pose(self):
        self._cancel_goal_publisher.publish(Message({"data": True}))
        return True

    def is_navigating(self):
        return self.context.get("/is_navigating")["data"]

    def get_remaining_nav_time(self):
        remaining_time = self.context.get("/navigation_remaining_time")
        if remaining_time is None:
            return 0
        else:
            return remaining_time["sec"]
