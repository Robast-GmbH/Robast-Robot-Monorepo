import logging
from logging.handlers import RotatingFileHandler
from threading import Lock, Timer
from models.cache_metadata import CacheMetadata
from models.timezone_formatter import TimezoneFormatter
import os
import time

LOG_FOLDER = "./log"
os.makedirs(LOG_FOLDER, exist_ok=True)


class Logger:
    def __init__(
        self,
        name: str,
        log_file: str,
        log_level=logging.INFO,
        max_file_size=1_000_000,
        backup_count=5,
        cache_timeout_in_s=0,
    ) -> None:
        """
        Initialize the Logger instance.

        Args:
            name (str): Name of the logger.
            log_file (str): File path for the log file.
            log_level (int): Logging level (e.g., logging.DEBUG, logging.INFO).
            max_file_size (int): Maximum size of a log file before rotation (in bytes).
            backup_count (int): Number of backup files to keep.
        """
        self.__logger = logging.getLogger(name)
        self.__logger.setLevel(log_level)

        self.__formatter = TimezoneFormatter(
            fmt="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%d-%m-%Y %H:%M:%S",
        )

        file_handler = RotatingFileHandler(
            f"{LOG_FOLDER}/{log_file}", maxBytes=max_file_size, backupCount=backup_count
        )
        file_handler.setFormatter(self.__formatter)
        self.__logger.addHandler(file_handler)

        self.__cache_timeout_in_s = cache_timeout_in_s
        if cache_timeout_in_s > 0:
            self.__cache: dict[str:CacheMetadata] = {}
            self.__lock = Lock()
            self.__process_timeout()

    def debug(self, message: str) -> None:
        """Log a debug-level message."""
        self.__logger.debug(message)

    def info(self, message: str) -> None:
        """Log an info-level message."""
        if self.__cache_timeout_in_s > 0:
            self.__add_message(message)
        else:
            self.__logger.info(message)

    def warning(self, message: str) -> None:
        """Log a warning-level message."""
        self.__logger.warning(message)

    def error(self, message: str) -> None:
        """Log an error-level message."""
        self.__logger.error(message)

    def critical(self, message: str) -> None:
        """Log a critical-level message."""
        self.__logger.critical(message)

    def __add_message(self, message: str) -> None:
        current_time = time.time()
        with self.__lock:
            if message in self.__cache:
                self.__cache[message].repetitions += 1
                self.__cache[message].last_timestamp = current_time
            else:
                self.__cache[message] = CacheMetadata(current_time)
                self.__logger.info(message)

    def __process_timeout(self) -> None:
        current_time = time.time()
        expired_messages = []

        with self.__lock:
            for message, cache_metadata in self.__cache.items():
                if (
                    current_time - cache_metadata.last_timestamp
                    > self.__cache_timeout_in_s
                ):
                    base_message = message.split(" ", 1)[1]
                    last_timestamp = self.__formatter.format_timestamp(
                        cache_metadata.last_timestamp
                    )
                    self.__logger.info(
                        f"timeout: {base_message}, last message {last_timestamp}, after {cache_metadata.repetitions} repetitions"
                    )
                    expired_messages.append(message)

            for message in expired_messages:
                del self.__cache[message]

        Timer(1, self.__process_timeout).start()
