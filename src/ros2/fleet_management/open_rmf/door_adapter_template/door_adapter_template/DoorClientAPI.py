import json
import socket
import time
import threading
from rmf_door_msgs.msg import DoorMode
import requests

class DoorClientAPI:
    def __init__(self,url,api_key,api_value,door_id):
        self.url = url
        self.header = {api_key:api_value}
        self.data = {"id": door_id}
        self.mode = DoorMode.MODE_CLOSED
        self.timer = None

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= 5:
                print("Unable to connect to door client API.")
                self.connected = False
                break
            else:
                print("Unable to connect to door client API. Attempting to reconnect...")
                count += 1
            time.sleep(1)

    def check_connection(self):
        ''' Return True if connection to the door API server is successful'''
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## ------------------------ ##
        return True

    def open_door(self):
        ''' Return True if the door API server is successful receive open door command'''
        response = requests.post("http://10.10.23.6:8003/open_door?robot_name=rb_theron")
        if response.status_code == 200:
            is_opening = response.json()["success"]
            if is_opening:
                self.start_timer(is_opening=True)
            return is_opening
        else:
            return False

    def start_timer(self, is_opening):
        self.mode = DoorMode.MODE_MOVING

        def set_mode(is_opening):
            if is_opening:
                self.mode = DoorMode.MODE_OPEN
            else:
                self.mode = DoorMode.MODE_CLOSED

        if self.timer:
            self.timer.cancel()
        self.timer = threading.Timer(5, set_mode, [is_opening])
        self.timer.start()

    def close_door(self):
        ''' Return True if the door API server is successful receive open door command'''
        response = requests.post("http://10.10.23.6:8003/close_door?robot_name=rb_theron")
        if response.status_code == 200:
            is_closing = response.json()["success"]
            if is_closing:
                self.start_timer(is_opening=False)
            return is_closing
        else:
            return False

    def get_mode(self):
        ''' Return the door status with reference rmf_door_msgs. 
            Return DoorMode.MODE_CLOSED when door status is closed.
            Return DoorMode.MODE_MOVING when door status is moving.
            Return DoorMode.MODE_OPEN when door status is open.
            Return DoorMode.MODE_OFFLINE when door status is offline.
            Return DoorMode.MODE_UNKNOWN when door status is unknown'''
        return self.mode
