from typing import Optional, Callable, Any
from roslibpy import Topic, Ros
from thread_safe_dict import ThreadSafeDict


class BaseBridge:
    def __init__(self, ros: Ros) -> None:
        self.context = ThreadSafeDict()
        self.ros = ros

    def _create_on_msg_callback(self, topic: str) -> Callable:
        def on_msg_callback(message: dict[str, Any]):
            self.context[topic] = message

        return on_msg_callback

    def start_subscriber(
        self, topic: str, msg_type: str, on_msg_callback: Optional[Callable] = None
    ) -> Topic:
        listener = Topic(self.ros, topic, msg_type)
        if on_msg_callback:
            listener.subscribe(on_msg_callback)
        else:
            listener.subscribe(self._create_on_msg_callback(topic))
        return listener

    def start_publisher(self, topic: str, msg_type: str) -> Topic:
        publisher = Topic(self.ros, topic, msg_type)
        return publisher
