import requests

DRAWER_IDLE = 0
DRAWER_OPEN = 1


class RobotDrawerAPI:
    def __init__(self, api_url) -> None:
        self.api_url = api_url
        self.robot_name = "rb_theron"
        self.module_id = 0
        self.drawer_id = 0

    def open_drawer(self, drawer_address):
        try:
            module_and_drawer_id = drawer_address.split("_")
            self.module_id = int(module_and_drawer_id[0])
            self.drawer_id = int(module_and_drawer_id[1])
            response = requests.post(
                f"{self.api_url}/open_drawer?robot_name={self.robot_name}&module_id={self.module_id}&drawer_id={self.drawer_id}"
            )
            response.raise_for_status()
        except Exception as e:
            print(f"Open drawer failed: {e}")

    def is_drawer_open(self):
        try:
            response = requests.get(
                f"{self.api_url}/modules?robot_name={self.robot_name}"
            )
            response.raise_for_status()
            is_open = response.json()[self.module_id - 1]["is_open"]
            return is_open
        except Exception as e:
            print(f"Request for drawer open status failed: {e}")
            return None
