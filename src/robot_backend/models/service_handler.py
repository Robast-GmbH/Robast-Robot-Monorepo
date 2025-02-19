from roslibpy import Ros, Service, ServiceRequest
from typing import Any
from models.logger import Logger


class ServiceHandler:
    def __init__(
        self,
        ros: Ros,
        name: str,
        service_type: str,
    ) -> None:
        logger_topic_name = name.lstrip("/").replace("/", "__")
        self.__logger = Logger(logger_topic_name, f"services/{logger_topic_name}.log")
        self.__service = Service(ros, name, service_type)

    def start_request(self, values: dict[str, Any]) -> Any | None:
        try:
            request = ServiceRequest(values=values)
            self.__logger.info(f"request: {values}")
            result = self.__service.call(request)
            self.__logger.info(f"result: {result}")
            return result
        except Exception as e:
            self.__logger.error(f"error: {e}")
            return None
