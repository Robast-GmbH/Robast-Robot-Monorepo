from sub_bridges.base_bridge import BaseBridge
from thread_safe_dict import ThreadSafeDict
from roslibpy import Ros, Message


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

    def navigate_to_goal_pose(self, robot_name, goal_pose):
        self.context["/is_navigating"] =  {"data": True}
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
        try:
            return self.context["/is_navigating"]["data"]
        except KeyError:
            return False

    def get_remaining_nav_time(self):
        try:
            return self.context["/navigation_remaining_time"]["sec"]
        except KeyError:
            return 0

