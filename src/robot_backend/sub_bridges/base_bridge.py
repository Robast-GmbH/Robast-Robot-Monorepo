from typing import Optional, Callable, Any
from roslibpy import Ros
from thread_safe_dict import ThreadSafeDict
from models.publisher import Publisher
from models.subscriber import Subscriber
from models.service_handler import ServiceHandler


class BaseBridge:
    def __init__(self, ros: Ros) -> None:
        self.context = ThreadSafeDict()
        self.__ros = ros

    def create_subscriber(
        self,
        topic: str,
        msg_type: str,
        on_msg_callback: Optional[Callable] = None,
        logger_cache_timeout_in_s: int = 0,
    ) -> Subscriber:
        return Subscriber(
            self.__ros,
            self.context,
            topic,
            msg_type,
            on_msg_callback,
            logger_cache_timeout_in_s,
        )

    def create_publisher(self, topic: str, msg_type: str) -> Publisher:
        return Publisher(self.__ros, topic, msg_type)

    def create_service(self, name: str, service_type: str) -> ServiceHandler:
        return ServiceHandler(self.__ros, name, service_type)
