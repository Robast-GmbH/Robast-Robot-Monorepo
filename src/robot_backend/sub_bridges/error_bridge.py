from sub_bridges.base_bridge import BaseBridge
from models.robast_error import RobastError
from roslibpy import Ros
from threading import Timer
from typing import Any, Dict, List
from collections import defaultdict


class ErrorBridge(BaseBridge):
    ERROR_MSG = "communication_interfaces/error_msgs/ErrorBaseMsg"
    ERROR_INVALIDATION_TIME_IN_S = 2.0

    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)

        self.__robast_error_subscriber = self.create_subscriber(
            "/robast_error_best_effort",
            ErrorBridge.ERROR_MSG,
            on_msg_callback=self.__on_error,
        )

        self.__error_by_id_by_code: Dict[int, Dict[str, RobastError]] = defaultdict(
            dict
        )

    def get_errors(self) -> Dict[str, List[Dict[str, Any]]]:
        return {
            str(error_code): self.__get_errors_by_code(error_code)
            for error_code in self.__error_by_id_by_code.keys()
            if self.__error_by_id_by_code[error_code]
        }

    def __on_error(self, msg: Dict[str, Any]) -> None:
        error = RobastError.from_dict(msg)
        self.__error_by_id_by_code[error.code][error.id] = error
        Timer(
            ErrorBridge.ERROR_INVALIDATION_TIME_IN_S,
            self.__invalidate_error,
            [error.code, error.id],
        ).start()

    def __invalidate_error(self, error_code: str, error_id: str) -> None:
        if (
            error_code in self.__error_by_id_by_code
            and error_id in self.__error_by_id_by_code[error_code]
        ):
            del self.__error_by_id_by_code[error_code][error_id]

    def __get_errors_by_code(self, error_code: str) -> List[Dict[str, Any]]:
        if error_code not in self.__error_by_id_by_code:
            return []
        return [
            error.to_dict() for error in self.__error_by_id_by_code[error_code].values()
        ]
