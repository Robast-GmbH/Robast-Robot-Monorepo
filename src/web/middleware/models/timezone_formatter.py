from logging import Formatter
from datetime import datetime
from zoneinfo import ZoneInfo


class TimezoneFormatter(Formatter):
    """
    Custom formatter to include timezone-aware timestamps.
    """

    def __init__(self, fmt=None, datefmt=None, tz="Europe/Berlin"):
        super().__init__(fmt=fmt, datefmt=datefmt)
        self.tz = ZoneInfo(tz)

    def formatTime(self, record, datefmt=None):
        record_time = datetime.fromtimestamp(record.created, tz=self.tz)
        if datefmt:
            return record_time.strftime(datefmt)
        return record_time.isoformat()
