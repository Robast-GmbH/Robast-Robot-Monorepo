from typing import Optional, Callable, Any
from roslibpy import Topic, Ros
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
