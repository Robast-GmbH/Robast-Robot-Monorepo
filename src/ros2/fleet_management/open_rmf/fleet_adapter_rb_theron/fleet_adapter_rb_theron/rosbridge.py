from thread_safe_dict import ThreadSafeDict

from roslibpy import Message, Ros, Topic


class Rosbridge:
    def __init__(self):
        self.context = ThreadSafeDict()
        self.ros = Ros("127.0.0.1", 9090)
        self.ros.run()

        self.start_subscriber(
            "/robot_position",
            "geometry_msgs/msg/Point",
        )
        self.start_subscriber(
            "/navigation_remaining_time",
            "builtin_interfaces/msg/Duration",
        )
        self.start_subscriber(
            "/is_navigating",
            "std_msgs/msg/Bool",
        )

        self._goal_pose_publisher = Topic(
            self.ros,
            "/set_goal_pose",
            "geometry_msgs/msg/Pose",
        )
        self._cancel_goal_publisher = Topic(
            self.ros,
            "/cancel_goal",
            "std_msgs/msg/Bool",
        )

        self.map_offset = [466.0, -179.4]
        self.map_scale = 0.05

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        return self.ros.is_connected

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        position = self.context.get("/robot_position")

        if position is None:
            return [self.map_offset[0], self.map_offset[1], 0.0]
        else:
            return [
                position["x"] / self.map_scale + self.map_offset[0],
                position["y"] / self.map_scale + self.map_offset[1],
                position["z"],
            ]

    def start_subscriber(self, topic, msg_type):
        listener = Topic(self.ros, topic, msg_type)
        listener.subscribe(lambda message: self.context.update(topic, message))

    def navigate_to_goal_pose(self, robot_name, goal_pose):
        self.context.update("/is_navigating", {"data": True})
        goal_msg = Message(
            {
                "position": {
                    "x": (goal_pose[0] - self.map_offset[0]) * self.map_scale,
                    "y": (goal_pose[1] - self.map_offset[1]) * self.map_scale,
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
        remaining_time = self.context.get("navigation_remaining_time")
        if remaining_time is None:
            return 0
        else:
            return remaining_time["sec"]
