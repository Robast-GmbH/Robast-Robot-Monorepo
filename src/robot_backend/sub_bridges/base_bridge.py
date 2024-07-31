from typing import Optional, Callable, Any
from roslibpy import Topic, Ros, Service, ServiceRequest
from thread_safe_dict import ThreadSafeDict


class BaseBridge:
    def __init__(self, ros: Ros) -> None:
        self.context = ThreadSafeDict()
        self.__ros = ros

    def __create_on_msg_callback(self, topic: str) -> Callable:
        def on_msg_callback(message: dict[str, Any]):
            self.context[topic] = message

        return on_msg_callback

    def start_subscriber(
        self, topic: str, msg_type: str, on_msg_callback: Optional[Callable] = None
    ) -> Topic:
        listener = Topic(self.__ros, topic, msg_type)
        if on_msg_callback:
            listener.subscribe(on_msg_callback)
        else:
            listener.subscribe(self.__create_on_msg_callback(topic))
        return listener

    def start_publisher(self, topic: str, msg_type: str) -> Topic:
        publisher = Topic(self.__ros, topic, msg_type)
        return publisher

    def start_service(
        self, name: str, service_type: str, values: dict[str, Any] | None = None
    ) -> Any:
        service = Service(self.__ros, name, service_type)
        request = ServiceRequest(values=values)
        result = service.call(request)
        self.context[name] = result
