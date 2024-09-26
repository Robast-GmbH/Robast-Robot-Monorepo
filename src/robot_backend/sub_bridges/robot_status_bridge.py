from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
from typing import Any


class RobotStatusBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.start_subscriber(
            "/robot/battery_estimator/data", "robotnik_msgs/msg/BatteryStatus"
        )

    def get_battery_status(self) -> dict[str, Any]:
        try:
            return {
                "status": "success",
                "data": self.context["/robot/battery_estimator/data"],
            }
        except KeyError:
            return {"status": "failure", "data": None}
