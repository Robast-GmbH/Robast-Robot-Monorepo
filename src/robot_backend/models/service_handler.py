from roslibpy import Ros, Service, ServiceRequest
from typing import Any
from thread_safe_dict import ThreadSafeDict
from models.logger import Logger


class ServiceHandler:
    def __init__(
        self,
        ros: Ros,
        context: ThreadSafeDict,
        name: str,
        service_type: str,
    ) -> None:
        logger_topic_name = name.lstrip("/").replace("/", "__")
        self.__logger = Logger(logger_topic_name, f"services/{logger_topic_name}.log")

        self.__name = name
        self.__context = context
        self.__service = Service(ros, name, service_type)

    def start_request(self, values: dict[str, Any]):
        request = ServiceRequest(values=values)
        self.__logger.info(f"request: {values}")
        result = self.__service.call(request)
        self.__logger.info(f"result: {result}")
        self.__context[self.__name] = result
