import threading


class ThreadSafeDict(dict):
    def __init__(self):
        self._lock = threading.Lock()
        self.dict = {}

    def __getitem__(self, key):
        with self._lock:
            return super().__getitem__(key)

    def __setitem__(self, key, value):
        with self._lock:
            super().__setitem__(key, value)

    def __delitem__(self, key):
        with self._lock:
            super().__delitem__(key)

    def values(self):
        with self._lock:
            return super().values()
