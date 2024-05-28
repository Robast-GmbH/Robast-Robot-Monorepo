import time
import threading
from rmf_door_msgs.msg import DoorMode
import requests


class DoorClientAPI:
    def __init__(self, url, api_key, api_value, door_id):
        self.__url = url
        self.__open_url = f"{self.__url}/open_door?robot_name=rb_theron"
        self.__close_url = f"{self.__url}/close_door?robot_name=rb_theron"
        self.__header = {api_key: api_value}
        self.__data = {"id": door_id}
        self.__mode = DoorMode.MODE_CLOSED
        self.__timer = None
        self.__door_delay_in_sec = 2

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
            response = requests.get(f"{self.__url}/")
            return response.status_code == 200
        except Exception as e:
            return False

    def open_door(self):
        """Return True if the door API server successfully received open door command"""
        return self.__trigger_door(DoorMode.MODE_OPEN, self.__open_url)

    def close_door(self):
        """Return True if the door API server successfully received close door command"""
        return self.__trigger_door(DoorMode.MODE_CLOSED, self.__close_url)

    def get_mode(self):
        """Return the door status with reference rmf_door_msgs.
        Return DoorMode.MODE_CLOSED when door status is closed.
        Return DoorMode.MODE_MOVING when door status is moving.
        Return DoorMode.MODE_OPEN when door status is open.
        Return DoorMode.MODE_OFFLINE when door status is offline.
        Return DoorMode.MODE_UNKNOWN when door status is unknown"""
        return self.__mode

    def __trigger_door(self, target_door_mode, url):
        triggered_successfully = False
        try:
            response = requests.get(url)
            if response.status_code == 200 and response.json()["success"]:
                self.__start_timer(target_door_mode=target_door_mode)
                triggered_successfully = True
        except Exception as e:
            self.__mode = DoorMode.MODE_OFFLINE

        return triggered_successfully

    def __start_timer(self, target_door_mode):
        def set_mode(target_door_mode):
            self.__mode = target_door_mode

        if self.__timer is None or not self.__timer.is_alive():
            self.__timer = threading.Timer(
                self.__door_delay_in_sec, set_mode, [target_door_mode]
            )
            self.__timer.start()
