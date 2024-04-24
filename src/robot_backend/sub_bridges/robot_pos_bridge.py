from sub_bridges.base_bridge import BaseBridge
from thread_safe_dict import ThreadSafeDict
from roslibpy import Ros


class RobotPosBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.start_subscriber("/robot_position", "geometry_msgs/msg/Point")

    def get_robot_pos(self):
        try:
            return self.context["/robot_position"]
        except KeyError:
            return {"x": 0.0, "y": 0.0, "z": 0.0}
