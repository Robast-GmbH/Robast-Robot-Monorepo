from roslibpy import Topic, Ros
from thread_safe_dict import ThreadSafeDict

class BaseBridge:
    def __init__(self,ros:Ros) -> None:
        self.context = ThreadSafeDict()
        self.ros = ros

    def start_subscriber(self, topic, msg_type,on_msg_callback=None)-> Topic:
        listener = Topic(self.ros, topic, msg_type)
        if on_msg_callback:
            listener.subscribe(on_msg_callback)
        else:
            listener.subscribe(lambda message: self.context.update(topic, message))
        return listener

    def start_publisher(self, topic, msg_type) -> Topic:
        publisher = Topic(self.ros, topic, msg_type)
        return publisher