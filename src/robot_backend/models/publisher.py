from roslibpy import Topic, Ros
from models.logger import Logger


class Publisher:
    def __init__(self, ros: Ros, topic: str, message_type: str) -> None:
        logger_topic_name = topic.lstrip("/").replace("/", "__")
        self.__logger = Logger(logger_topic_name, f"publisher/{logger_topic_name}.log")
        self.__topic = Topic(ros, topic, message_type)

    def publish(self, message: dict[str, any]) -> None:
        self.__logger.info(f"published: {message}")
        self.__topic.publish(message)
