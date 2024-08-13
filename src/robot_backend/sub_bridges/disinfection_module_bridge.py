from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
import threading
from typing import Any, Dict


disinfection_triggered = threading.Event()


class DisinfectionModuleBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.start_subscriber(
            "/disinfection_triggered",
            "builtin_interfaces/msg/Time",
            on_msg_callback=self.__on_disinfection_triggered,
        )

    def wait_for_disinfection_triggered(self, time_out: int) -> Dict[str, str]:
        disinfection_triggered.clear()
        was_successful = disinfection_triggered.wait(timeout=float(time_out))
        return {"status": "success" if was_successful else "failure"}

    def __on_disinfection_triggered(self, msg: dict[str, Any]) -> None:
        disinfection_triggered.set()
