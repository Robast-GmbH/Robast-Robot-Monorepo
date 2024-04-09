from roslibpy import Topic, Ros
from thread_safe_dict import ThreadSafeDict

class BaseBridge:
    def __init__(self,ros:Ros,context:ThreadSafeDict) -> None:
        self.ros = ros
        self.context = context

    def start_subscriber(self, topic, msg_type)-> Topic:
        listener = Topic(self.ros, topic, msg_type)
        listener.subscribe(lambda message: self.context.update(topic, message))
        return listener

    def start_publisher(self, topic, msg_type) -> Topic:
        publisher = Topic(self.ros, topic, msg_type)
        return publisher