from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
from typing import Any


class RobotStatusBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.create_subscriber(
            "/robot/battery_estimator/data", "robotnik_msgs/msg/BatteryStatus"
        )
        self.create_subscriber(
            "/robot/safety_module/emergency_stop",
            "std_msgs/msg/Bool",
            logger_cache_timeout_in_s=15,
        )

    def get_battery_status(self) -> dict[str, Any]:
        try:
            return {
                "status": "success",
                "data": self.context["/robot/battery_estimator/data"],
            }
        except KeyError:
            return {"status": "failure", "data": None}

    def get_emergency_stop_pressed(self) -> dict[str, Any]:
        try:
            return {
                "status": "success",
                "data": self.context["/robot/safety_module/emergency_stop"]["data"],
            }
        except KeyError:
            return {"status": "failure", "data": None}
