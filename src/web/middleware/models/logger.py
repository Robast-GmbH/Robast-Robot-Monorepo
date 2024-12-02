import logging
from logging.handlers import RotatingFileHandler
from models.timezone_formatter import TimezoneFormatter


class Logger:
    def __init__(
        self,
        name: str,
        log_file: str,
        log_level=logging.INFO,
        max_file_size=1_000_000,
        backup_count=5,
    ):
        """
        Initialize the Logger instance.

        Args:
            name (str): Name of the logger.
            log_file (str): File path for the log file.
            log_level (int): Logging level (e.g., logging.DEBUG, logging.INFO).
            max_file_size (int): Maximum size of a log file before rotation (in bytes).
            backup_count (int): Number of backup files to keep.
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(log_level)

        formatter = TimezoneFormatter(
            fmt="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%d-%m-%Y %H:%M:%S",
        )

        file_handler = RotatingFileHandler(
            log_file, maxBytes=max_file_size, backupCount=backup_count
        )
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

    def debug(self, message: str):
        """Log a debug-level message."""
        self.logger.debug(message)

    def info(self, message: str):
        """Log an info-level message."""
        self.logger.info(message)

    def warning(self, message: str):
        """Log a warning-level message."""
        self.logger.warning(message)

    def error(self, message: str):
        """Log an error-level message."""
        self.logger.error(message)

    def critical(self, message: str):
        """Log a critical-level message."""
        self.logger.critical(message)
