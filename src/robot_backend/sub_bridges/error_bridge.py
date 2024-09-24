from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
from threading import Timer
from typing import Any, Dict


class ErrorBridge(BaseBridge):
    ERROR_MSG = "communication_interfaces/error_msgs/ErrorBaseMsg"
    ERROR_INVALIDATION_TIME_IN_S = 2.0
    DRAWER_NOT_OPENED_ERROR_CODE = 50301

    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)

        self.__robast_error_subscriber = self.start_subscriber(
            "/robast_error",
            ErrorBridge.ERROR_MSG,
            on_msg_callback=self.__on_error,
        )

        self.__error_clear_timers: Dict[int, Timer] = {}

    def received_drawer_not_opened_error(self) -> bool:
        return self.DRAWER_NOT_OPENED_ERROR_CODE in self.__error_clear_timers

    def __on_error(self, msg: Dict[str, Any]) -> None:
        error_code = msg["error_code"]

        if error_code in self.__error_clear_timers:
            self.__error_clear_timers[error_code].cancel()

        timer = Timer(
            ErrorBridge.ERROR_INVALIDATION_TIME_IN_S,
            self.__invalidate_error,
            [error_code],
        )
        self.__error_clear_timers[error_code] = timer
        timer.start()

    def __invalidate_error(self, error_code: int) -> None:
        if error_code in self.__error_clear_timers:
            del self.__error_clear_timers[error_code]
