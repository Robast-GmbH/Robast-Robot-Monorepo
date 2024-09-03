from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
from typing import Any


class RobotPosBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.start_subscriber("/robot_position", "geometry_msgs/msg/Point")
        self.start_subscriber("/is_robot_lost", "std_msgs/msg/Bool")
        self.__set_initial_point_publisher = self.start_publisher(
            "/set_initial_point", "geometry_msgs/msg/Point"
        )

    def get_robot_pos(self) -> dict[str, float]:
        try:
            return self.context["/robot_position"]
        except KeyError:
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    def get_robot_lost(self) -> dict[str, Any]:
        try:
            return {"status": "success", "data": self.context["/is_robot_lost"]["data"]}
        except KeyError:
            return {"status": "failure", "data": False}

    def set_initial_point(self, x: float, y: float, z: float) -> None:
        self.__set_initial_point_publisher.publish({"x": x, "y": y, "z": z})
