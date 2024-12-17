from logging import Formatter
from datetime import datetime
from zoneinfo import ZoneInfo


class TimezoneFormatter(Formatter):
    def __init__(self, fmt=None, datefmt=None, tz="Europe/Berlin"):
        super().__init__(fmt=fmt, datefmt=datefmt)
        self.__timezone = ZoneInfo(tz)

    def format_timestamp(self, timestamp: float):
        timestamp_with_timezone = datetime.fromtimestamp(timestamp, tz=self.__timezone)
        return timestamp_with_timezone.strftime("%d-%m-%Y %H:%M:%S")

    def formatTime(self, record, datefmt=None):
        record_time = datetime.fromtimestamp(record.created, tz=self.__timezone)
        if datefmt:
            return record_time.strftime(datefmt)
        return record_time.isoformat()
