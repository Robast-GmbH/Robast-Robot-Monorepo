import threading
from typing import Callable, Any


class PeriodicTimer:
    def __init__(
        self, interval: float, callback: Callable[[], None], *args: Any, **kwargs: Any
    ):
        self.interval = interval
        self.callback = callback
        self.args = args
        self.kwargs = kwargs
        self.timer = None
        self.running = False

    def _run(self) -> None:
        if self.running:
            self.callback(*self.args, **self.kwargs)
            self.running = False
            self.start()

    def start(self) -> None:
        if not self.running:
            self.running = True
            self.timer = threading.Timer(self.interval, self._run)
            self.timer.start()

    def stop(self) -> None:
        self.running = False
        if self.timer:
            self.timer.cancel()
            self.timer = None
