from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
import threading
from typing import Any, Dict
import os

disinfection_triggered = threading.Event()


class DisinfectionModuleBridge(BaseBridge):
    FULL = 650

    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.start_subscriber(
            "/disinfection_triggered",
            "builtin_interfaces/msg/Time",
            on_msg_callback=self.__on_disinfection_triggered,
        )
        home_directory = os.path.expanduser("~")
        self.__file_path = f"{home_directory}/log/remaining_disinfections.txt"

    def wait_for_disinfection_triggered(self, time_out: int) -> Dict[str, str]:
        disinfection_triggered.clear()
        was_successful = disinfection_triggered.wait(timeout=float(time_out))
        return {"status": "success" if was_successful else "failure"}

    def refill_disinfection_fluid_container(self) -> Dict[str, str]:
        if self.__write_remaining_disinfections(DisinfectionModuleBridge.FULL):
            return {"status": "success"}
        else:
            return {"status": "failure"}

    def get_disinfection_module_status(self) -> Dict[str, int]:
        remaining_disinfections = self.__read_remaining_disinfections()
        if remaining_disinfections is not None:
            return {
                "status": "success",
                "remaining_disinfections": remaining_disinfections,
                "max_disinfections": DisinfectionModuleBridge.FULL,
            }
        return {"status": "failure"}

    def __write_remaining_disinfections(self, remaining_disinfections: int) -> bool:
        if remaining_disinfections < 0:
            return False
        try:
            os.makedirs(os.path.dirname(self.__file_path), exist_ok=True)
            with open(self.__file_path, "w") as file:
                file.write(str(remaining_disinfections))
            return True
        except (PermissionError, OSError, TypeError):
            return False

    def __read_remaining_disinfections(self) -> int | None:
        try:
            with open(self.__file_path, "r") as file:
                return int(file.read().strip())
        except FileNotFoundError:
            self.__write_remaining_disinfections(0)
            return 0
        except (PermissionError, OSError, TypeError, ValueError):
            return None

    def __on_disinfection_triggered(self, msg: dict[str, Any]) -> None:
        disinfection_triggered.set()
        remaining_disinfections = self.__read_remaining_disinfections()
        if remaining_disinfections is None:
            return
        if remaining_disinfections == 0:
            return
        remaining_disinfections -= 1
        self.__write_remaining_disinfections(remaining_disinfections)
