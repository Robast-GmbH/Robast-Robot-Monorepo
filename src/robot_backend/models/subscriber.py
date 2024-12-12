from typing import Optional, Callable, Any
from roslibpy import Ros, Topic
from thread_safe_dict import ThreadSafeDict
from models.logger import Logger


class Subscriber:
    def __init__(
        self,
        ros: Ros,
        context: ThreadSafeDict,
        topic: str,
        message_type: str,
        on_msg_callback: Optional[Callable] = None,
        logger_cache_timeout_in_s: int = 0,
    ) -> None:
        logger_topic_name = f'{topic.lstrip("/").replace("/", "__")}'
        self.__logger = Logger(
            logger_topic_name,
            f"subscriber/{logger_topic_name}.log",
            cache_timeout_in_s=logger_cache_timeout_in_s,
        )

        self.__context = context
        self.__topic = self.__start_subscriber(
            ros, topic, message_type, on_msg_callback
        )

    def __start_subscriber(
        self,
        ros: Ros,
        topic: str,
        msg_type: str,
        on_msg_callback: Optional[Callable] = None,
    ) -> Topic:
        topic = Topic(ros, topic, msg_type)
        if on_msg_callback:
            on_msg_callback_with_log = self.__add_subscriber_log_callback(
                on_msg_callback
            )
            topic.subscribe(on_msg_callback_with_log)
        else:
            on_msg_callback_with_log = self.__add_subscriber_log_callback(
                self.__create_on_msg_callback(topic)
            )
            topic.subscribe(on_msg_callback_with_log)
        return topic

    def __create_on_msg_callback(self, topic: str) -> Callable:
        def on_msg_callback(message: dict[str, Any]):
            self.__context[topic] = message

        return on_msg_callback

    def __add_subscriber_log_callback(self, on_msg: Callable) -> Callable:
        def log_callback(message: dict[str, Any]):
            self.__logger.info(f"received: {message}")
            on_msg(message)

        return log_callback
