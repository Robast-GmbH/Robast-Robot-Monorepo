import time
import threading
from rmf_door_msgs.msg import DoorMode
import requests


class DoorClientAPI:
    def __init__(self, url, api_key, api_value, door_id):
        self._url = url
        self._open_url = f"{self._url}/open_door?robot_name=rb_theron"
        self._close_url = f"{self._url}/close_door?robot_name=rb_theron"
        self._header = {api_key: api_value}
        self._data = {"id": door_id}
        self._mode = DoorMode.MODE_CLOSED
        self._timer = None
        self.door_delay = 2 # seconds

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= 5:
                print("Unable to connect to door client API.")
                self.connected = False
                break
            else:
                print(
                    "Unable to connect to door client API. Attempting to reconnect..."
                )
                count += 1
            time.sleep(1)

    def check_connection(self):
        """Return True if connection to the door API server is successful"""
        try:
            response = requests.get(f'{self._url}/')
            return response.status_code == 200
        except Exception as e:
            return False

    def open_door(self):
        """Return True if the door API server successfully received open door command"""
        return self._trigger_door(DoorMode.MODE_OPEN, self._open_url)

    def close_door(self):
        """Return True if the door API server successfully received close door command"""
        return self._trigger_door(DoorMode.MODE_CLOSED, self._close_url)

    def get_mode(self):
        """Return the door status with reference rmf_door_msgs.
        Return DoorMode.MODE_CLOSED when door status is closed.
        Return DoorMode.MODE_MOVING when door status is moving.
        Return DoorMode.MODE_OPEN when door status is open.
        Return DoorMode.MODE_OFFLINE when door status is offline.
        Return DoorMode.MODE_UNKNOWN when door status is unknown"""
        return self._mode

    def _trigger_door(self, target_door_mode, url):
        triggered_successfully = False
        try:
            response = requests.get(url)
            if response.status_code == 200 and response.json()["success"]:
                self._start_timer(target_door_mode=target_door_mode)
                triggered_successfully = True
        except Exception as e:
            self._mode = DoorMode.MODE_OFFLINE

        return triggered_successfully

    def _start_timer(self, target_door_mode):
        def set_mode(target_door_mode):
            self._mode = target_door_mode

        if self._timer is None or not self._timer.is_alive():
            self._timer = threading.Timer(self.door_delay, set_mode, [target_door_mode])
            self._timer.start()
